"""Regression test for the teleop bug shipped in 0.4.1.

Symptom: after switching the server to the multi-arm builder, joints became
prefixed (``arm/Joint_1``), but the teleop client had ``"Joint_1"`` hardcoded
and crashed in ``TCPSolver.__init__`` with KeyError.

Fix: the server reports the prefixed names in its ``hello`` reply, and the
client uses them. This test fails on the old behaviour and passes on the fix.
"""
from __future__ import annotations

import time

import mujoco
import pytest

from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.control.workcell import ArmInstance, build_multi_arm_model
from loophole_arm.server.cli import (
    _build_endpoints,
    _default_scene_for,
    _save_model_for_clients,
)
from loophole_arm.server.remote_backend import RemoteBackend
from loophole_arm.server.sim_server import SimServer


@pytest.fixture
def running_server():
    arms = [ArmInstance(name="arm", mount_pos=(0.0, 0.0, 0.10))]
    scene = _default_scene_for(arms)
    model, spec, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    endpoints = _build_endpoints(model, data, handles)
    server = SimServer(model=model, data=data, endpoints=endpoints,
                       host="127.0.0.1", port=9881,
                       model_path=_save_model_for_clients(spec))
    server._start_tcp_listener()
    yield server
    server.shutdown()
    time.sleep(0.05)


def test_remote_backend_reports_prefixed_names(running_server) -> None:
    """The names the server reports must be usable by TCPSolver — i.e. they
    must match what the compiled model actually contains."""
    client = RemoteBackend("arm", port=9881)
    try:
        client.connect()
        # Names came back populated.
        assert client.arm_joint_names, "server did not report arm_joint_names"
        assert client.tcp_site, "server did not report tcp_site"
        assert client.gripper_actuator, "server did not report gripper_actuator"
        # And they are the prefixed names (this is what the bug was about).
        assert all(n.startswith("arm/") for n in client.arm_joint_names)
        assert client.tcp_site.startswith("arm/")

        # The decisive check: TCPSolver built with these names + the remote
        # kinematic_model must NOT raise. This is exactly what teleop does.
        model = client.kinematic_model()
        solver = TCPSolver(model, client.tcp_site, arm_joint_names=client.arm_joint_names)
        assert solver is not None
    finally:
        client.disconnect()
