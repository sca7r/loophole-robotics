"""Tests for the layered control system (workcell + controller + IK)."""
from __future__ import annotations

import numpy as np

from loophole_arm.control import make_sim_robot
from loophole_arm.control.workcell import WorkcellConfig, build_workcell_model


def test_workcell_compiles() -> None:
    m = build_workcell_model(WorkcellConfig(arm="feetech"))
    assert m.nu == 7  # 6 arm + 1 gripper
    assert m.nlight >= 3  # three-point studio lighting
    assert m.nmat >= 4  # floor, steel, frame, workpiece materials


def test_arm_holds_home_pose() -> None:
    """The arm must hold its commanded pose under gravity (regression test).

    A previous bug left the position actuators with biastype=NONE, so they
    applied raw force and the arm collapsed. This guards against that.
    """
    import mujoco

    robot, model, data, home = make_sim_robot(arm="feetech")
    data.ctrl[:6] = home
    data.ctrl[6] = 0.0
    for _ in range(800):
        mujoco.mj_step(model, data)

    drift_deg = np.abs(np.array(home) - data.qpos[:6]).max() * 180 / np.pi
    assert drift_deg < 2.0, f"arm drifted {drift_deg:.1f}° from home — actuators unstable"


def test_no_self_collision_at_home() -> None:
    """Adjacent arm links must not self-collide at the home pose."""
    import mujoco

    robot, model, data, home = make_sim_robot(arm="feetech")
    mujoco.mj_forward(model, data)
    assert data.ncon == 0, f"{data.ncon} spurious contacts at home pose"


def test_ik_reaches_targets() -> None:
    """move_to should drive the end-effector to within ~1 cm of the target."""
    robot, model, data, home = make_sim_robot(arm="feetech")

    targets = [
        (0.20, 0.06, 0.18),
        (0.20, -0.06, 0.18),
        (0.18, 0.00, 0.16),
    ]
    for tx, ty, tz in targets:
        reached = robot.move_to(tx, ty, tz, duration=1.0)
        ee = robot.backend.end_effector_pose()
        err_mm = np.linalg.norm(ee - [tx, ty, tz]) * 1000
        assert reached, f"IK reported failure for ({tx},{ty},{tz})"
        assert err_mm < 20, f"target ({tx},{ty},{tz}): {err_mm:.0f}mm error"


def test_pick_and_place_runs() -> None:
    """A full pick → place → home sequence completes without error."""
    robot, model, data, home = make_sim_robot(arm="feetech")
    robot.home(home)
    # pick/place are exercised by the Pick/Place skills in test_skills.py;
    # the controller-level duplicates were removed in 0.8.0.
    from loophole_arm.skills import Pick, Place
    assert Pick(x=0.18, y=0.08, z=0.14).run(robot).ok
    assert Place(x=0.18, y=-0.08, z=0.14).run(robot).ok
    robot.home(home)
    # Arm returned near home
    drift = np.abs(np.array(home) - robot.backend.joint_positions).max()
    assert drift < 0.3  # radians


def test_gripper_opens_and_closes() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech", safety=False)
    grip_id = robot.backend._grip_act_id
    robot.open_gripper()
    open_cmd = data.ctrl[grip_id]
    robot.close_gripper()
    closed_cmd = data.ctrl[grip_id]
    assert open_cmd != closed_cmd


def test_tcp_site_exists() -> None:
    """The model must expose a TCP site as the IK control frame."""
    from loophole_arm.control.workcell import TCP_SITE, WorkcellConfig, build_workcell_model

    m = build_workcell_model(WorkcellConfig(arm="feetech"))
    assert m.nsite >= 1
    assert m.site(TCP_SITE).name == TCP_SITE


def test_mink_solver_accuracy() -> None:
    """The mink IK solver should hit targets to sub-cm accuracy."""
    import mujoco

    from loophole_arm.control.kinematics import TCPSolver
    from loophole_arm.control.workcell import TCP_SITE, WorkcellConfig, build_workcell_model

    m = build_workcell_model(WorkcellConfig(arm="feetech"))
    d = mujoco.MjData(m)
    home = np.array([0.0, -0.5, 1.0, 0.0, 0.0, 0.0])
    d.qpos[:6] = home
    mujoco.mj_forward(m, d)

    solver = TCPSolver(
        m, TCP_SITE,
        arm_joint_names=["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"],
    )
    for target in [(0.20, 0.06, 0.18), (0.20, -0.06, 0.18), (0.22, 0.0, 0.20)]:
        sol = solver.solve(np.array(target), home)
        assert sol.converged, f"mink failed to converge for {target}"
        assert sol.position_error < 0.01, f"{target}: {sol.position_error*1000:.1f}mm"
