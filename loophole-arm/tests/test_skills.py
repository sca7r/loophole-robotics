"""Skill Engine tests: the 11 PRD skills, the registry, and taught points.

Uses a real SimBackend + RobotController against a single-arm scene, so
skill success/failure reflects genuine motion + safety behaviour.
"""
from __future__ import annotations

import mujoco
import pytest

from loophole_arm.control.controller import RobotController
from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.control.limits import SafetyLimits
from loophole_arm.control.safety import SafetyBackend
from loophole_arm.control.scene import Scene
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import ArmInstance, build_multi_arm_model
from loophole_arm.skills import (
    CloseGripper,
    Delay,
    ExecuteSkill,
    Home,
    MoveJoint,
    MoveLinear,
    OpenGripper,
    Pick,
    Place,
    Repeat,
    Wait,
)
from loophole_arm.skills.base import SkillStatus
from loophole_arm.skills.engine import SkillEngine, SkillNotFoundError


@pytest.fixture(scope="module")
def robot() -> RobotController:
    scene = Scene().add_table(size=(0.35, 0.45), height=0.10, pos=(0.0, 0.0))
    arms = [ArmInstance(name="t", mount_pos=(0.0, 0.0, 0.10))]
    model, _, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    h = handles[0]
    sim = SimBackend(model=model, data=data, arm_joint_names=h.arm_joints,
                     gripper_actuator=h.gripper_actuator, tcp_site=h.tcp_site)
    backend = SafetyBackend(sim, SafetyLimits.feetech_default())
    backend.connect()
    backend.enable()
    solver = TCPSolver(model, h.tcp_site, arm_joint_names=h.arm_joints)
    ctrl = RobotController(backend=backend, solver=solver, control_hz=20.0,
                           home_pose=h.home)
    ctrl.home(list(h.home))
    return ctrl


# ── Individual skills ────────────────────────────────────────────────────
def test_home_skill(robot) -> None:
    assert Home().run(robot).ok


def test_move_joint_skill(robot) -> None:
    r = MoveJoint(joints=(0.1, -0.5, 1.0, 0.0, 0.0, 0.0), duration=0.5).run(robot)
    assert r.ok
    assert abs(robot.backend.joint_positions[0] - 0.1) < 0.05


def test_move_joint_rejects_wrong_count(robot) -> None:
    r = MoveJoint(joints=(0.0, 0.0), duration=0.5).run(robot)  # type: ignore[arg-type]
    assert r.status == SkillStatus.REJECTED


def test_move_linear_skill(robot) -> None:
    r = MoveLinear(x=0.20, y=0.05, z=0.20, duration=0.8).run(robot)
    assert r.ok
    tcp = robot.backend.end_effector_pose()
    assert abs(tcp[0] - 0.20) < 0.02 and abs(tcp[1] - 0.05) < 0.02


def test_move_linear_rejects_unreachable(robot) -> None:
    r = MoveLinear(x=2.0, y=0.0, z=0.2).run(robot)   # 2 m — far outside envelope
    assert r.status == SkillStatus.REJECTED


def test_gripper_skills(robot) -> None:
    assert OpenGripper().run(robot).ok
    assert CloseGripper().run(robot).ok


def test_wait_and_delay(robot) -> None:
    assert Wait(0.05).run(robot).ok
    assert Delay(0.05).run(robot).ok
    assert Wait(0.0).run(robot).status == SkillStatus.SKIPPED


def test_pick_and_place_skills(robot) -> None:
    Home().run(robot)
    assert Pick(x=0.20, y=0.00, z=0.14, descend_duration=0.6, lift_duration=0.6).run(robot).ok
    assert Place(x=0.20, y=-0.08, z=0.14, descend_duration=0.6, lift_duration=0.6).run(robot).ok


def test_repeat_skill(robot) -> None:
    r = Repeat(skill=Wait(0.02), times=3).run(robot)
    assert r.ok and r.data["iterations"] == 3
    assert Repeat(skill=None).run(robot).status == SkillStatus.REJECTED
    assert Repeat(skill=Wait(0.01), times=0).run(robot).status == SkillStatus.SKIPPED


def test_execute_skill_via_registry(robot) -> None:
    engine = SkillEngine()
    engine.register("go_home", Home())
    r = ExecuteSkill(skill_name="go_home", engine=engine).run(robot)
    assert r.ok
    r = ExecuteSkill(skill_name="nope", engine=engine).run(robot)
    assert r.status == SkillStatus.REJECTED


# ── Engine: sequences + taught points ────────────────────────────────────
def test_sequence_stops_at_failure(robot) -> None:
    engine = SkillEngine()
    trace = engine.run_sequence(
        [Home(), MoveLinear(x=2.0, y=0, z=0.2), Wait(0.1)], robot
    )
    # Home ok, MoveLinear rejected, Wait never ran.
    assert len(trace) == 2
    assert trace[0].ok and trace[1].status == SkillStatus.REJECTED


def test_teach_point_roundtrip(robot, tmp_path) -> None:
    engine = SkillEngine()
    Home().run(robot)
    p = engine.teach_point("home_pose", robot)
    assert p.name == "home_pose" and len(p.joints) == 6

    path = tmp_path / "points.json"
    engine.save_points(path)

    fresh = SkillEngine()
    assert fresh.load_points(path) == 1
    loaded = fresh.get_point("home_pose")
    assert loaded.joints == pytest.approx(p.joints)
    assert loaded.tcp == pytest.approx(p.tcp)

    with pytest.raises(SkillNotFoundError):
        fresh.get_point("nope")


def test_pick_at_taught_point(robot) -> None:
    """The full industry workflow: move somewhere, teach it, pick at the name."""
    engine = SkillEngine()
    MoveLinear(x=0.20, y=0.0, z=0.15, duration=0.8).run(robot)
    p = engine.teach_point("part_a", robot)
    result = engine.run(Pick(x=p.tcp[0], y=p.tcp[1], z=p.tcp[2],
                             descend_duration=0.6, lift_duration=0.6), robot)
    assert result.ok
