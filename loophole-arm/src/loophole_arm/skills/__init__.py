"""Skill Engine — reusable, parameterised robot primitives.

A *skill* is a self-contained unit of robot work with a clear contract:
inputs go in, the robot moves, and a typed result comes out. Skills compose:
``Pick(at=p1) → Place(at=p2)`` is a sequence the user writes by name, not by
manually computing joint angles or Cartesian deltas.

This is the layer the PRD calls "Skill Engine" — and the layer customers will
actually write programs against. They never call ``move_to(x, y, z)`` directly.

Design intent:

* Skills target :class:`RobotController` (which targets :class:`RobotInterface`),
  so the **same skill sequence runs against the sim backend, the remote
  backend, and the hardware backend with zero changes**. That's the entire
  point of the architecture; the Skill Engine is what makes it visible.

* No coordinate math leaks into application code. Approach/retreat heights are
  *parameters* of the skill, not numbers the user computes.

* Skills are dataclasses for introspection (the FSM logs them, the recorder
  saves them, future Behavior Tree nodes consume them).
"""
from loophole_arm.skills.base import (
    Skill,
    SkillResult,
    SkillStatus,
)
from loophole_arm.skills.engine import (
    SkillEngine,
    SkillNotFoundError,
)
from loophole_arm.skills.library import (
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

__all__ = [
    "CloseGripper",
    "Delay",
    "ExecuteSkill",
    "Home",
    "MoveJoint",
    "MoveLinear",
    "OpenGripper",
    "Pick",
    "Place",
    "Repeat",
    "Skill",
    "SkillEngine",
    "SkillNotFoundError",
    "SkillResult",
    "SkillStatus",
    "Wait",
]
