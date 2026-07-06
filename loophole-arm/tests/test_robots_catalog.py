"""Robot catalog tests: robots/<name>/robot.yaml is the single source of truth."""
from __future__ import annotations

import pytest

from loophole_arm.robots import (
    RobotNotFoundError,
    available_robots,
    load_robot,
    robots_dir,
)


def test_available_robots_finds_folders() -> None:
    names = available_robots()
    assert "feetech" in names
    assert "ur5e" in names


def test_feetech_spec_complete() -> None:
    s = load_robot("feetech")
    assert s.n_joints == 6
    assert s.joints == ("Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6")
    assert s.gripper_actuator == "Joint_Gripper"
    assert s.gripper_dof == 1
    assert len(s.home) == 6
    assert len(s.motors) == 6
    assert s.model_format == "urdf"
    assert s.model_path.exists(), "URDF must live inside robots/feetech/"
    assert s.meshes_path is not None and s.meshes_path.exists()
    # The whole point of the restructure: model files live in the robot folder.
    assert robots_dir() / "feetech" in s.model_path.parents


def test_ur5e_paths_resolve_from_repo_root() -> None:
    s = load_robot("ur5e")
    # Vendored model: path points into assets/menagerie (fetched separately),
    # so we only assert the path shape, not existence.
    assert "menagerie" in str(s.model_path)
    assert s.hardware.get("bus") == "ur_rtde"


def test_unknown_robot_raises() -> None:
    with pytest.raises(RobotNotFoundError, match="available"):
        load_robot("does_not_exist")
