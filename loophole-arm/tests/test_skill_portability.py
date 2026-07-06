"""The portability promise, proven by test.

The SAME list of Skill objects runs against (a) a local SimBackend and (b) a
RemoteBackend talking to a sim server over TCP — with zero changes to the
skill list. Both runs must succeed and end at the same TCP position.

This is the PRD's core success metric: "A pick-and-place application can run
unchanged in simulation and on hardware." The RemoteBackend exercises the
same RobotInterface seam that HardwareBackend sits behind, so passing here
demonstrates the seam works; the hardware backend swap is config, not code.
"""
from __future__ import annotations

import time

import mujoco
import numpy as np

from loophole_arm.control.controller import RobotController
from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.control.limits import SafetyLimits
from loophole_arm.control.safety import SafetyBackend
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import ArmInstance, build_multi_arm_model
from loophole_arm.server.cli import (
    _build_endpoints,
    _default_scene_for,
    _save_model_for_clients,
)
from loophole_arm.server.remote_backend import RemoteBackend
from loophole_arm.server.sim_server import SimServer
from loophole_arm.skills import CloseGripper, Home, MoveLinear, OpenGripper, Wait
from loophole_arm.skills.engine import SkillEngine

# THE application: one skill list, written once, used against both backends.
PROGRAM = [
    Home(),
    MoveLinear(x=0.20, y=0.05, z=0.18, duration=0.8),
    CloseGripper(),
    Wait(0.05),
    MoveLinear(x=0.20, y=-0.05, z=0.18, duration=0.8),
    OpenGripper(),
]
FINAL_TCP = (0.20, -0.05, 0.18)


def _assert_program_runs(robot: RobotController) -> None:
    engine = SkillEngine()
    trace = engine.run_sequence(PROGRAM, robot)
    assert len(trace) == len(PROGRAM), (
        f"program stopped early at step {len(trace)}: {trace[-1].detail}"
    )
    assert all(r.ok or r.status.value == "skipped" for r in trace)
    tcp = robot.backend.end_effector_pose()
    assert np.allclose(tcp, FINAL_TCP, atol=0.03), f"final TCP {tcp} != {FINAL_TCP}"


def test_program_runs_on_local_sim() -> None:
    arms = [ArmInstance(name="arm", mount_pos=(0.0, 0.0, 0.10))]
    scene = _default_scene_for(arms)
    model, _, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    h = handles[0]
    sim = SimBackend(model=model, data=data, arm_joint_names=h.arm_joints,
                     gripper_actuator=h.gripper_actuator, tcp_site=h.tcp_site)
    backend = SafetyBackend(sim, SafetyLimits.feetech_default())
    backend.connect()
    backend.enable()
    solver = TCPSolver(model, h.tcp_site, arm_joint_names=h.arm_joints)
    robot = RobotController(backend=backend, solver=solver, control_hz=20.0,
                            home_pose=h.home)
    _assert_program_runs(robot)


def test_same_program_runs_over_the_wire() -> None:
    # Server side — exactly what loophole-armd builds.
    arms = [ArmInstance(name="arm", mount_pos=(0.0, 0.0, 0.10))]
    scene = _default_scene_for(arms)
    model, spec, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    endpoints = _build_endpoints(model, data, handles)
    server = SimServer(model=model, data=data, endpoints=endpoints,
                       host="127.0.0.1", port=9893,
                       model_path=_save_model_for_clients(spec))
    server._start_tcp_listener()
    try:
        client = RemoteBackend("arm", port=9893)
        client.connect()
        solver = TCPSolver(client.kinematic_model(), client.tcp_site,
                           arm_joint_names=client.arm_joint_names)
        robot = RobotController(backend=client, solver=solver, control_hz=20.0,
                                home_pose=client.home_pose)
        robot.enable()

        # RemoteBackend.step() sleeps wall-clock; the server has no physics
        # thread in this test, so we pump physics from a helper thread the
        # same way the live server's physics loop does.
        import threading
        stop = threading.Event()

        def pump() -> None:
            while not stop.is_set():
                with server._state_lock:
                    mujoco.mj_step(model, data)
                time.sleep(0.001)

        t = threading.Thread(target=pump, daemon=True)
        t.start()
        try:
            _assert_program_runs(robot)   # <-- the SAME PROGRAM list
        finally:
            stop.set()
            t.join(timeout=1.0)
            client.disconnect()
    finally:
        server.shutdown()
        time.sleep(0.05)
