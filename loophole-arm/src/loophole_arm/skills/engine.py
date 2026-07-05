"""SkillEngine — named-skill registry, sequence runner, and taught-point store.

The engine is the operator-facing surface of the Skill Engine layer:

* **Registry** — register a skill under a name, run it later by name
  (:class:`ExecuteSkill` uses this).
* **Sequence runner** — run a list of skills in order, stop at the first
  non-OK result, return the full trace for logging/diagnosis.
* **Taught points** — named poses the operator records at the teach prompt
  ("teach pick_pose") and later references by name ("pick pick_pose").
  This is the industry teach-pendant pattern: operators think in named
  points, never in raw coordinates.

The engine holds no backend state of its own — every ``run`` takes the
:class:`RobotController` explicitly, so the same engine instance (and the same
saved points file) works against sim, remote, and hardware backends.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from loophole_arm.skills.base import Skill, SkillResult, SkillStatus

if TYPE_CHECKING:
    from loophole_arm.control.controller import RobotController

logger = logging.getLogger("loophole_arm.skills")


class SkillNotFoundError(KeyError):
    """Raised when a named skill is not in the registry."""


@dataclass(frozen=True)
class TaughtPoint:
    """A named pose recorded at the teach prompt.

    Both representations are stored: joints are exact (replayable even if IK
    changes), the TCP position is human-meaningful (printable, offsettable).
    """
    name: str
    joints: tuple[float, ...]
    tcp: tuple[float, float, float]


@dataclass
class SkillEngine:
    """Registry + runner + taught-point store."""

    _skills: dict[str, Skill] = field(default_factory=dict)
    _points: dict[str, TaughtPoint] = field(default_factory=dict)

    # ── Skill registry ──────────────────────────────────────────────────
    def register(self, name: str, skill: Skill) -> None:
        """Register (or replace) a named skill."""
        if not name:
            raise ValueError("skill name cannot be empty")
        self._skills[name] = skill
        logger.debug("registered skill %r: %s", name, skill.describe())

    def get(self, name: str) -> Skill:
        if name not in self._skills:
            raise SkillNotFoundError(
                f"no skill named {name!r}; registered: {sorted(self._skills)}"
            )
        return self._skills[name]

    def names(self) -> list[str]:
        return sorted(self._skills)

    # ── Execution ───────────────────────────────────────────────────────
    def run(self, skill: Skill, robot: RobotController) -> SkillResult:
        """Run one skill, logging start and outcome."""
        logger.info("skill start: %s", skill.describe())
        result = skill.run(robot)
        if result.ok:
            logger.info("skill ok: %s", skill.describe())
        else:
            logger.warning("skill %s: %s — %s",
                           result.status.value, skill.describe(), result.detail)
        return result

    def run_sequence(
        self, skills: list[Skill], robot: RobotController
    ) -> list[SkillResult]:
        """Run skills in order; stop at the first REJECTED/FAILED.

        Returns the trace of everything that ran (including the failure),
        so callers can log or display exactly where a program stopped.
        """
        trace: list[SkillResult] = []
        for skill in skills:
            result = self.run(skill, robot)
            trace.append(result)
            if result.status in (SkillStatus.REJECTED, SkillStatus.FAILED):
                logger.warning("sequence stopped at %s (%d/%d)",
                               skill.describe(), len(trace), len(skills))
                break
        return trace

    # ── Taught points ───────────────────────────────────────────────────
    def teach_point(self, name: str, robot: RobotController) -> TaughtPoint:
        """Record the robot's *current* pose under ``name``."""
        if not name:
            raise ValueError("point name cannot be empty")
        joints = tuple(float(v) for v in robot.backend.joint_positions)
        tcp = robot.backend.end_effector_pose()
        point = TaughtPoint(name=name, joints=joints,
                            tcp=(float(tcp[0]), float(tcp[1]), float(tcp[2])))
        self._points[name] = point
        logger.info("taught point %r at TCP (%.3f, %.3f, %.3f)", name, *point.tcp)
        return point

    def get_point(self, name: str) -> TaughtPoint:
        if name not in self._points:
            raise SkillNotFoundError(
                f"no taught point named {name!r}; taught: {sorted(self._points)}"
            )
        return self._points[name]

    def point_names(self) -> list[str]:
        return sorted(self._points)

    # ── Persistence (taught points survive across sessions) ────────────
    def save_points(self, path: str | Path) -> str:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format_version": 1,
            "points": [
                {"name": p.name, "joints": list(p.joints), "tcp": list(p.tcp)}
                for p in self._points.values()
            ],
        }
        path.write_text(json.dumps(payload, indent=2))
        logger.info("saved %d taught points to %s", len(self._points), path)
        return str(path)

    def load_points(self, path: str | Path) -> int:
        """Load taught points from disk (merging over existing). Returns count."""
        path = Path(path)
        if not path.exists():
            return 0
        data = json.loads(path.read_text())
        for entry in data.get("points", []):
            p = TaughtPoint(
                name=str(entry["name"]),
                joints=tuple(float(v) for v in entry["joints"]),
                tcp=tuple(float(v) for v in entry["tcp"]),  # type: ignore[arg-type]
            )
            self._points[p.name] = p
        logger.info("loaded %d taught points from %s", len(data.get("points", [])), path)
        return len(data.get("points", []))
