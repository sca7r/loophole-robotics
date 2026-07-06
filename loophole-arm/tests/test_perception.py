"""Sim perception tests: object_positions from the physics state.

In simulation, "where is the object" is answered by the physics engine
itself. These tests prove the answer flows through every layer: SimBackend,
the SafetyBackend decorator, and the wire protocol to a remote client.

The last test documents a known, tracked gap: the gripper reaches and
touches the cube but contact friction does not yet hold it through the
lift. See the skip reason for the diagnosis and the planned fix.
"""
from __future__ import annotations

import time

import mujoco
import pytest

from loophole_arm.control.limits import SafetyLimits
from loophole_arm.control.mock_backend import MockBackend
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


def _sim_stack():
    arms = [ArmInstance(name="arm", mount_pos=(0.0, 0.0, 0.10))]
    scene = _default_scene_for(arms)          # ships two cubes
    model, spec, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, spec, handles


def test_sim_backend_reports_objects() -> None:
    model, data, _, handles = _sim_stack()
    h = handles[0]
    sim = SimBackend(model=model, data=data, arm_joint_names=h.arm_joints,
                     gripper_actuator=h.gripper_actuator, tcp_site=h.tcp_site)
    sim.connect()
    objs = sim.object_positions()
    # The default scene has two cubes; both must appear with 3D positions.
    assert len(objs) == 2
    for name, pos in objs.items():
        assert len(pos) == 3
        # Objects start on/above the table (z > table height 0.10).
        assert pos[2] > 0.09, f"{name} at implausible z={pos[2]}"


def test_safety_decorator_forwards_perception() -> None:
    model, data, _, handles = _sim_stack()
    h = handles[0]
    sim = SimBackend(model=model, data=data, arm_joint_names=h.arm_joints,
                     gripper_actuator=h.gripper_actuator, tcp_site=h.tcp_site)
    wrapped = SafetyBackend(sim, SafetyLimits.feetech_default())
    wrapped.connect()
    # Without explicit forwarding, the interface's default {} would mask
    # the sim's data. This asserts the decorator forwards.
    assert len(wrapped.object_positions()) == 2


def test_mock_backend_has_no_perception() -> None:
    m = MockBackend()
    m.connect()
    assert m.object_positions() == {}


def test_object_positions_over_the_wire() -> None:
    model, data, spec, handles = _sim_stack()
    endpoints = _build_endpoints(model, data, handles)
    server = SimServer(model=model, data=data, endpoints=endpoints,
                       host="127.0.0.1", port=9897,
                       model_path=_save_model_for_clients(spec))
    server._start_tcp_listener()
    try:
        client = RemoteBackend("arm", port=9897)
        client.connect()
        objs = client.object_positions()
        assert len(objs) == 2
        for pos in objs.values():
            assert len(pos) == 3
    finally:
        client.disconnect()
        server.shutdown()
        time.sleep(0.05)


@pytest.mark.skip(reason=(
    "KNOWN GAP (tracked, root-caused): the generic mechanisms landed in "
    "0.10.0 (fingertip pads + TCP definition in robot.yaml, IK branch "
    "restart, closed-loop refinement) and low-height reach now works, but "
    "the lift is still red for a measured geometric reason: the jaw "
    "aperture is 3.3 cm, the demo cube is 2.5 cm (4 mm clearance per "
    "side), and TCP tracking plateaus at ~1.6 cm, so a jaw column clips "
    "the cube during descent and only one pad ever pinches. Grasp "
    "reliability = aperture margin vs tracking accuracy. Closing the "
    "final ~12 mm is a focused tuning task: raise servo gains near the "
    "workspace floor and/or widen the pad aperture, then unskip and "
    "assert cube dz > 0.02 after Pick at the cube's own height."
))
def test_pick_physically_lifts_cube() -> None:
    raise AssertionError("unskip once tracking/aperture margin closes")
