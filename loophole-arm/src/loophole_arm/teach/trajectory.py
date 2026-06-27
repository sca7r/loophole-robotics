"""The trajectory — a taught skill, saved to disk and replayed.

A :class:`Trajectory` is an ordered list of :class:`Waypoint` steps. It is the
single artifact that flows through the whole product:

    teach  →  Trajectory  →  save to .json  →  load  →  repeat (sim or hardware)

Because a waypoint is expressed in robot-agnostic terms (joint angles, a
Cartesian target, a gripper fraction) and is replayed through the same
``RobotController`` + ``RobotInterface`` in both worlds, **a skill taught in
simulation replays on hardware unchanged.** That is the core promise of
teach-and-repeat, and the reason the format stays high-level and editable
rather than a dense dump of motor counts.

The on-disk format is plain JSON: human-readable, diff-able, hand-editable.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

WaypointKind = Literal["joint", "cartesian", "gripper", "dwell"]

FORMAT_VERSION = "1.0"


@dataclass
class Waypoint:
    """One taught step.

    Exactly one of the payload fields is meaningful, selected by ``kind``:

    ===========  =========================================================
    kind         payload
    ===========  =========================================================
    ``joint``     ``joints`` — absolute joint angles (radians)
    ``cartesian`` ``position`` — TCP target ``[x, y, z]`` (metres); IK solves
    ``gripper``   ``gripper`` — closed fraction, 0.0 open … 1.0 closed
    ``dwell``     (none) — just pause for ``duration`` seconds
    ===========  =========================================================

    ``duration`` is the move time (for motion steps) or the pause length (for
    ``dwell``). ``label`` is an optional human note shown during replay.
    """

    kind: WaypointKind
    joints: list[float] | None = None
    position: list[float] | None = None
    gripper: float | None = None
    duration: float = 1.5
    label: str = ""

    def __post_init__(self) -> None:
        if self.kind == "joint" and self.joints is None:
            raise ValueError("joint waypoint requires `joints`")
        if self.kind == "cartesian" and self.position is None:
            raise ValueError("cartesian waypoint requires `position`")
        if self.kind == "gripper" and self.gripper is None:
            raise ValueError("gripper waypoint requires `gripper`")


@dataclass
class Trajectory:
    """An ordered, named sequence of taught waypoints."""

    name: str
    arm: str
    control_hz: float = 20.0
    waypoints: list[Waypoint] = field(default_factory=list)
    format_version: str = FORMAT_VERSION
    created: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())

    # ── Editing ─────────────────────────────────────────────────────────
    def add(self, wp: Waypoint) -> None:
        self.waypoints.append(wp)

    def __len__(self) -> int:
        return len(self.waypoints)

    # ── Persistence ─────────────────────────────────────────────────────
    def to_dict(self) -> dict:
        return {
            "format_version": self.format_version,
            "name": self.name,
            "arm": self.arm,
            "control_hz": self.control_hz,
            "created": self.created,
            "waypoints": [asdict(wp) for wp in self.waypoints],
        }

    @classmethod
    def from_dict(cls, data: dict) -> Trajectory:
        version = data.get("format_version", "0")
        if version.split(".")[0] != FORMAT_VERSION.split(".")[0]:
            raise ValueError(
                f"incompatible trajectory format {version!r}; expected {FORMAT_VERSION}"
            )
        traj = cls(
            name=data["name"],
            arm=data["arm"],
            control_hz=data.get("control_hz", 20.0),
            format_version=version,
            created=data.get("created", ""),
        )
        for wp in data["waypoints"]:
            traj.add(Waypoint(**wp))
        return traj

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2))
        return path

    @classmethod
    def load(cls, path: str | Path) -> Trajectory:
        return cls.from_dict(json.loads(Path(path).read_text()))
