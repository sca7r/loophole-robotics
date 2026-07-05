"""The :class:`Skill` ABC and its result types.

Every skill in the library inherits from :class:`Skill` and implements
:meth:`Skill.run`. The result is always a :class:`SkillResult` with a
:class:`SkillStatus` — *never* an exception for expected failure modes
(unreachable target, safety reject). That keeps composition predictable: a
sequence runner can decide whether to retry, recover, or abort based on the
status without try/except trees.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from loophole_arm.control.controller import RobotController


class SkillStatus(enum.Enum):
    """Outcome of a single skill execution.

    OK
        The skill completed as intended.
    SKIPPED
        The skill was not applicable (e.g. ``Wait`` with ``duration=0``).
    REJECTED
        The skill could not run safely — target outside workspace, joint
        limits violated, e-stop engaged. The robot stayed put.
    FAILED
        The skill ran but did not achieve its goal (IK didn't converge,
        target unreachable, motion timed out).
    """
    OK = "ok"
    SKIPPED = "skipped"
    REJECTED = "rejected"
    FAILED = "failed"


@dataclass(frozen=True)
class SkillResult:
    """What a skill returns. Never an exception for expected outcomes."""
    status: SkillStatus
    skill_name: str
    detail: str = ""
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.status == SkillStatus.OK

    @classmethod
    def make_ok(cls, name: str, **data: Any) -> SkillResult:
        return cls(SkillStatus.OK, name, data=dict(data))

    @classmethod
    def make_rejected(cls, name: str, detail: str) -> SkillResult:
        return cls(SkillStatus.REJECTED, name, detail=detail)

    @classmethod
    def make_failed(cls, name: str, detail: str) -> SkillResult:
        return cls(SkillStatus.FAILED, name, detail=detail)

    @classmethod
    def make_skipped(cls, name: str, detail: str = "") -> SkillResult:
        return cls(SkillStatus.SKIPPED, name, detail=detail)


@dataclass(frozen=True)
class Skill:
    """Abstract base for all skills. Subclasses are frozen dataclasses
    themselves (input parameters as fields) and override :meth:`run`.

    The required name is the class's ``__name__`` by default; subclasses
    override ``name`` only if they need to differ.
    """

    @property
    def name(self) -> str:
        return type(self).__name__

    def run(self, robot: RobotController) -> SkillResult:
        """Execute the skill. Subclasses must implement.

        Implementations should:
        * Validate parameters before motion.
        * Return ``SkillResult.make_rejected/failed`` on expected error modes.
        * Raise only on programming bugs (e.g. wrong type passed).
        """
        raise NotImplementedError(
            f"{type(self).__name__}.run is abstract"
        )

    def describe(self) -> str:
        """One-line description for logs and the FSM trace."""
        return self.name
