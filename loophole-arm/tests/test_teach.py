"""Tests for the teach-and-repeat product."""
from __future__ import annotations

import numpy as np

from loophole_arm.control import make_sim_robot
from loophole_arm.teach import TeachSession, Trajectory, TrajectoryPlayer, Waypoint


# ── Trajectory data model ────────────────────────────────────────────────
def test_waypoint_validation() -> None:
    import pytest

    with pytest.raises(ValueError, match="joint waypoint requires"):
        Waypoint(kind="joint")
    with pytest.raises(ValueError, match="cartesian waypoint requires"):
        Waypoint(kind="cartesian")
    with pytest.raises(ValueError, match="gripper waypoint requires"):
        Waypoint(kind="gripper")
    # dwell needs no payload
    Waypoint(kind="dwell", duration=1.0)


def test_trajectory_roundtrip(tmp_path) -> None:
    traj = Trajectory(name="t", arm="feetech")
    traj.add(Waypoint(kind="joint", joints=[0, 0, 0, 0, 0, 0], label="a"))
    traj.add(Waypoint(kind="cartesian", position=[0.2, 0.0, 0.2], label="b"))
    traj.add(Waypoint(kind="gripper", gripper=1.0))
    traj.add(Waypoint(kind="dwell", duration=0.5))

    path = traj.save(tmp_path / "t.json")
    loaded = Trajectory.load(path)
    assert loaded.name == "t"
    assert loaded.arm == "feetech"
    assert len(loaded) == 4
    assert loaded.waypoints[1].position == [0.2, 0.0, 0.2]
    assert loaded.waypoints[2].gripper == 1.0


def test_incompatible_format_rejected() -> None:
    import pytest

    with pytest.raises(ValueError, match="incompatible trajectory format"):
        Trajectory.from_dict({"format_version": "99.0", "name": "x", "arm": "feetech",
                              "waypoints": []})


# ── Teach session ────────────────────────────────────────────────────────
def test_capture_records_current_pose() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech")
    session = TeachSession(robot, name="cap", arm="feetech")
    robot.home(home)
    wp = session.capture(label="home")
    assert wp.kind == "joint"
    assert wp.joints is not None
    assert np.allclose(wp.joints, home, atol=0.05)
    assert len(session.trajectory) == 1


def test_teach_cartesian_moves_and_records() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech")
    session = TeachSession(robot, name="c", arm="feetech")
    robot.home(home)
    ok = session.teach_cartesian(0.18, 0.08, 0.18, label="pick")
    assert ok
    assert len(session.trajectory) == 1
    # The arm actually moved to (close to) the target.
    tcp = robot.backend.end_effector_pose()
    assert np.linalg.norm(tcp - [0.18, 0.08, 0.18]) < 0.02


def test_teach_cartesian_rejects_unreachable() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech")
    session = TeachSession(robot, name="c", arm="feetech")
    # Far outside the workspace envelope.
    ok = session.teach_cartesian(0.9, 0.0, 0.5)
    assert not ok
    assert len(session.trajectory) == 0  # nothing recorded


# ── Teach → save → load → repeat ─────────────────────────────────────────
def test_full_teach_and_repeat(tmp_path) -> None:
    robot, model, data, home = make_sim_robot(arm="feetech")
    session = TeachSession(robot, name="pp", arm="feetech")
    robot.home(home)
    session.teach_joints(home, label="home")
    session.teach_cartesian(0.18, 0.08, 0.18, label="above")
    session.teach_cartesian(0.18, 0.08, 0.12, label="grasp")
    session.teach_gripper(1.0)
    session.teach_cartesian(0.18, 0.08, 0.18, label="lift")
    path = session.save(tmp_path / "pp.json")

    # Reload in a fresh robot and replay.
    robot2, model2, data2, home2 = make_sim_robot(arm="feetech")
    traj = Trajectory.load(path)
    assert traj.arm == "feetech"
    player = TrajectoryPlayer(robot2)
    assert player.play(traj)
    # Arm ended near the final taught waypoint (lift position).
    tcp = robot2.backend.end_effector_pose()
    assert np.linalg.norm(tcp - [0.18, 0.08, 0.18]) < 0.03


def test_playback_loops() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech")
    session = TeachSession(robot, name="loop", arm="feetech")
    robot.home(home)
    session.teach_joints(home)
    session.teach_cartesian(0.18, 0.0, 0.18)
    player = TrajectoryPlayer(robot)
    assert player.play(session.trajectory, loops=2)
