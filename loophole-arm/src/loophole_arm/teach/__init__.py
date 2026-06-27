"""loophole-arm teach — teach-and-repeat by waypoints.

The first Loophole Arm product. Teach a skill by setting waypoints in the
simulator; save it as a portable trajectory; replay it on demand — in sim now,
on the physical arm later, unchanged.

    from loophole_arm.control import make_sim_robot
    from loophole_arm.teach import TeachSession, TrajectoryPlayer, Trajectory

    robot, model, data, home = make_sim_robot(arm="feetech")

    # Teach
    session = TeachSession(robot, name="pick_place", arm="feetech")
    session.teach_cartesian(0.18, 0.08, 0.18, label="above pick")
    session.teach_cartesian(0.18, 0.08, 0.12, label="grasp")
    session.teach_gripper(1.0, label="close")
    session.save("skills/pick_place.json")

    # Repeat (later, or on hardware)
    traj = Trajectory.load("skills/pick_place.json")
    TrajectoryPlayer(robot).play(traj)
"""
from loophole_arm.teach.player import TrajectoryPlayer
from loophole_arm.teach.session import TeachSession
from loophole_arm.teach.trajectory import FORMAT_VERSION, Trajectory, Waypoint

__all__ = [
    "FORMAT_VERSION",
    "TeachSession",
    "Trajectory",
    "TrajectoryPlayer",
    "Waypoint",
]
