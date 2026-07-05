"""The 11 skills required by the PRD.

Every skill is a frozen dataclass: its inputs are fields, its execution is the
``run(robot)`` method, and it returns a :class:`SkillResult` rather than
raising on expected failure modes. This makes skills introspectable (logs,
recorder, FSM) and composable (`Pick → Place → Home` is a list of dataclasses).

**Critically:** every skill targets :class:`RobotController`, which targets
:class:`RobotInterface`. That means the same skill instance runs against the
sim backend, the remote backend, and the hardware backend with zero changes.
That is the entire portability promise of the architecture, made concrete.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from loophole_arm.skills.base import Skill, SkillResult

if TYPE_CHECKING:
    from loophole_arm.control.controller import RobotController
    from loophole_arm.skills.engine import SkillEngine


# ── Motion primitives ───────────────────────────────────────────────────


@dataclass(frozen=True)
class Home(Skill):
    """Move the arm to its home pose. The pose itself is a property of the
    robot, not the skill — the skill just asks the robot to home."""

    pose: tuple[float, ...] | None = None

    def run(self, robot: RobotController) -> SkillResult:
        home = list(self.pose) if self.pose is not None else [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
        ok = robot.home(home)
        return SkillResult.make_ok(self.name) if ok else SkillResult.make_failed(self.name, "home motion failed")

    def describe(self) -> str:
        return f"Home(pose={self.pose})" if self.pose else "Home"


@dataclass(frozen=True)
class MoveJoint(Skill):
    """Drive all six arm joints to absolute targets (radians)."""

    joints: tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0)
    duration: float = 1.0

    def run(self, robot: RobotController) -> SkillResult:
        if len(self.joints) != 6:
            return SkillResult.make_rejected(self.name, f"need 6 joint targets, got {len(self.joints)}")
        ok = robot.move_joints(list(self.joints), duration=self.duration)
        return SkillResult.make_ok(self.name) if ok else SkillResult.make_failed(self.name, "joint move failed")

    def describe(self) -> str:
        return f"MoveJoint({[round(v, 3) for v in self.joints]}, {self.duration:.2f}s)"


@dataclass(frozen=True)
class MoveLinear(Skill):
    """Move the TCP to (x, y, z) metres, IK-solved by the controller."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    duration: float = 1.0

    def run(self, robot: RobotController) -> SkillResult:
        ok = robot.move_to(self.x, self.y, self.z, duration=self.duration)
        if not ok:
            return SkillResult.make_rejected(self.name,
                                             f"target ({self.x:.3f}, {self.y:.3f}, {self.z:.3f}) "
                                             "unreachable or outside workspace")
        return SkillResult.make_ok(self.name)

    def describe(self) -> str:
        return f"MoveLinear({self.x:+.3f}, {self.y:+.3f}, {self.z:+.3f})"


# ── Gripper ─────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class OpenGripper(Skill):
    """Open the gripper fully."""

    def run(self, robot: RobotController) -> SkillResult:
        robot.open_gripper()
        return SkillResult.make_ok(self.name)


@dataclass(frozen=True)
class CloseGripper(Skill):
    """Close the gripper fully."""

    def run(self, robot: RobotController) -> SkillResult:
        robot.close_gripper()
        return SkillResult.make_ok(self.name)


# ── Pick / Place templates ──────────────────────────────────────────────


@dataclass(frozen=True)
class Pick(Skill):
    """Industry-style pick template: approach → descend → close → lift.

    The operator just gives a target XYZ. The approach height and segment
    durations are parameters (defaults match a 5 cm Z-clearance pick, which
    is the de-facto industry default for tabletop work).
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    approach_height: float = 0.05      # 5 cm — UR/ABB default
    descend_duration: float = 1.0
    lift_duration: float = 1.0
    settle_seconds: float = 0.15       # let the gripper close before lifting

    def run(self, robot: RobotController) -> SkillResult:
        # 1. Approach: move above the target.
        ok = robot.move_to(self.x, self.y, self.z + self.approach_height,
                           duration=self.descend_duration)
        if not ok:
            return SkillResult.make_rejected(self.name, "approach pose unreachable")
        # 2. Open the gripper before descending.
        robot.open_gripper()
        # 3. Descend.
        ok = robot.move_to(self.x, self.y, self.z, duration=self.descend_duration)
        if not ok:
            return SkillResult.make_rejected(self.name, "descend pose unreachable")
        # 4. Grasp.
        robot.close_gripper()
        time.sleep(self.settle_seconds)
        # 5. Lift.
        ok = robot.move_to(self.x, self.y, self.z + self.approach_height,
                           duration=self.lift_duration)
        if not ok:
            return SkillResult.make_failed(self.name, "lift pose unreachable after grasp")
        return SkillResult.make_ok(self.name)

    def describe(self) -> str:
        return f"Pick(({self.x:+.3f}, {self.y:+.3f}, {self.z:+.3f}), approach={self.approach_height*100:.0f}cm)"


@dataclass(frozen=True)
class Place(Skill):
    """Industry-style place template: approach → descend → open → lift.

    Mirror of :class:`Pick`. Approach from above, lower, release, retreat.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    approach_height: float = 0.05
    descend_duration: float = 1.0
    lift_duration: float = 1.0
    settle_seconds: float = 0.15

    def run(self, robot: RobotController) -> SkillResult:
        ok = robot.move_to(self.x, self.y, self.z + self.approach_height,
                           duration=self.descend_duration)
        if not ok:
            return SkillResult.make_rejected(self.name, "approach pose unreachable")
        ok = robot.move_to(self.x, self.y, self.z, duration=self.descend_duration)
        if not ok:
            return SkillResult.make_rejected(self.name, "descend pose unreachable")
        robot.open_gripper()
        time.sleep(self.settle_seconds)
        ok = robot.move_to(self.x, self.y, self.z + self.approach_height,
                           duration=self.lift_duration)
        if not ok:
            return SkillResult.make_failed(self.name, "retreat unreachable after release")
        return SkillResult.make_ok(self.name)

    def describe(self) -> str:
        return f"Place(({self.x:+.3f}, {self.y:+.3f}, {self.z:+.3f}), approach={self.approach_height*100:.0f}cm)"


# ── Timing ──────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Wait(Skill):
    """Pause for ``duration`` seconds before the next skill."""

    duration: float = 0.5

    def run(self, robot: RobotController) -> SkillResult:
        if self.duration <= 0:
            return SkillResult.make_skipped(self.name, "non-positive duration")
        time.sleep(self.duration)
        return SkillResult.make_ok(self.name)


@dataclass(frozen=True)
class Delay(Skill):
    """Alias for :class:`Wait`. The PRD lists both; we expose both."""

    duration: float = 0.5

    def run(self, robot: RobotController) -> SkillResult:
        return Wait(self.duration).run(robot)


# ── Composition ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Repeat(Skill):
    """Run a sub-skill ``times`` times. Stops at the first non-OK result."""

    skill: Skill | None = None
    times: int = 1

    def run(self, robot: RobotController) -> SkillResult:
        if self.skill is None:
            return SkillResult.make_rejected(self.name, "no sub-skill provided")
        if self.times <= 0:
            return SkillResult.make_skipped(self.name, "non-positive repeat count")
        for i in range(self.times):
            r = self.skill.run(robot)
            if not r.ok and r.status.name != "SKIPPED":
                return SkillResult.make_failed(
                    self.name, f"sub-skill failed at iteration {i+1}/{self.times}: {r.detail}"
                )
        return SkillResult.make_ok(self.name, iterations=self.times)

    def describe(self) -> str:
        sub = self.skill.describe() if self.skill else "<none>"
        return f"Repeat({sub}, x{self.times})"


@dataclass(frozen=True)
class ExecuteSkill(Skill):
    """Look up a named skill in a :class:`SkillEngine` registry and run it.

    This is how the FSM and behavior-tree layers will call user-defined
    skills by name, without holding a direct Python reference.
    """

    skill_name: str = ""
    engine: SkillEngine | None = field(default=None, repr=False)

    def run(self, robot: RobotController) -> SkillResult:
        if self.engine is None or not self.skill_name:
            return SkillResult.make_rejected(
                self.name, "engine and skill_name are required"
            )
        try:
            target = self.engine.get(self.skill_name)
        except KeyError as e:
            return SkillResult.make_rejected(self.name, str(e))
        return target.run(robot)

    def describe(self) -> str:
        return f"ExecuteSkill({self.skill_name!r})"
