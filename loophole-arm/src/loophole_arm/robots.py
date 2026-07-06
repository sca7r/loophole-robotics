"""Robot catalog: one folder per robot, one YAML per robot, one loader.

Every robot lives in ``robots/<name>/`` at the repository root. The folder
contains the model files (URDF or MJCF plus meshes) and a ``robot.yaml``
describing everything the software needs: joint names, home pose, gripper
actuator, motor channels, hardware defaults.

Code never hardcodes these facts. It calls::

    from loophole_arm.robots import load_robot
    spec = load_robot("feetech")
    spec.joints        # ["Joint_1", ..., "Joint_6"]
    spec.home          # [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
    spec.model_path    # absolute Path to the URDF/MJCF

Adding a new robot means adding a folder, not editing source.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

# robots/ sits at the repo root, three levels up from this file
# (src/loophole_arm/robots.py -> src/loophole_arm -> src -> root).
_ROBOTS_DIR = Path(__file__).resolve().parents[2] / "robots"


class RobotNotFoundError(KeyError):
    """Raised when no robots/<name>/robot.yaml exists."""


@dataclass(frozen=True)
class RobotSpec:
    """Parsed robot.yaml plus resolved absolute paths."""

    name: str
    description: str
    joints: tuple[str, ...]
    home: tuple[float, ...]
    gripper_actuator: str
    gripper_dof: int
    motors: tuple[str, ...]
    model_format: str            # "urdf" or "mjcf"
    model_path: Path             # absolute
    meshes_path: Path | None     # absolute, or None if the model has none
    hardware: dict = field(default_factory=dict)
    actuation: dict = field(default_factory=dict)   # tuning: kp gains, force limits
    collision: dict = field(default_factory=dict)   # grasp pads etc., see robot.yaml
    tcp_parent: str = ""                            # body carrying the TCP site
    tcp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def n_joints(self) -> int:
        return len(self.joints)


def robots_dir() -> Path:
    """The robots/ directory (exposed for tests and tooling)."""
    return _ROBOTS_DIR


def available_robots() -> list[str]:
    """Names of every robot folder that has a robot.yaml."""
    if not _ROBOTS_DIR.exists():
        return []
    return sorted(
        p.parent.name for p in _ROBOTS_DIR.glob("*/robot.yaml")
    )


def load_robot(name: str) -> RobotSpec:
    """Load robots/<name>/robot.yaml into a :class:`RobotSpec`."""
    folder = _ROBOTS_DIR / name
    yaml_path = folder / "robot.yaml"
    if not yaml_path.exists():
        raise RobotNotFoundError(
            f"no robot named {name!r}; available: {available_robots()}"
        )
    data = yaml.safe_load(yaml_path.read_text()) or {}

    model = data.get("model") or {}
    model_file = str(model.get("file", ""))
    if not model_file:
        raise ValueError(f"{yaml_path}: model.file is required")
    # Paths in robot.yaml are relative to the robot folder, except entries
    # that explicitly start with "assets/" which resolve from the repo root
    # (used for vendored models like the Menagerie UR5e).
    root = _ROBOTS_DIR.parent
    model_path = (root / model_file) if model_file.startswith("assets/") else (folder / model_file)

    meshes = model.get("meshes")
    meshes_path: Path | None = None
    if meshes:
        m = str(meshes)
        meshes_path = (root / m) if m.startswith("assets/") else (folder / m)

    gripper = data.get("gripper") or {}
    joints = tuple(str(j) for j in (data.get("joints") or []))
    home = tuple(float(v) for v in (data.get("home") or []))
    if joints and home and len(joints) != len(home):
        raise ValueError(
            f"{yaml_path}: home has {len(home)} values but there are {len(joints)} joints"
        )

    return RobotSpec(
        name=str(data.get("name", name)),
        description=str(data.get("description", "")),
        joints=joints,
        home=home,
        gripper_actuator=str(gripper.get("actuator", "")),
        gripper_dof=int(gripper.get("dof", 1)),
        motors=tuple(str(m) for m in (data.get("motors") or [])),
        model_format=str(model.get("format", "urdf")),
        model_path=model_path,
        meshes_path=meshes_path,
        hardware=dict(data.get("hardware") or {}),
        actuation=dict(data.get("actuation") or {}),
        collision=dict(data.get("collision") or {}),
        tcp_parent=str((data.get("tcp") or {}).get("parent", "")),
        tcp_offset=tuple(float(v) for v in (data.get("tcp") or {}).get("offset", [0, 0, 0])),
    )
