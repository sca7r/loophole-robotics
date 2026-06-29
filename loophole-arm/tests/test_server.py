"""Tests for the sim-server / RemoteBackend / wire protocol."""
from __future__ import annotations

import time

import mujoco
import numpy as np
import pytest

from loophole_arm.control.workcell import ArmInstance, build_multi_arm_model
from loophole_arm.server.cli import (
    _build_endpoints,
    _default_scene_for,
    _save_model_for_clients,
)
from loophole_arm.server.protocol import (
    PROTOCOL_VERSION,
    Request,
    Response,
    parse_request,
    parse_response,
    serialise_request,
    serialise_response,
    versions_compatible,
)
from loophole_arm.server.remote_backend import RemoteBackend, RemoteConnectionError
from loophole_arm.server.sim_server import SimServer


# ── Protocol round-trip ──────────────────────────────────────────────────
def test_request_roundtrip() -> None:
    req = Request(op="send_joint_targets", robot="arm_a",
                  args={"targets": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]})
    line = serialise_request(req)
    parsed = parse_request(line)
    assert parsed.op == "send_joint_targets"
    assert parsed.robot == "arm_a"
    assert parsed.args["targets"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert parsed.version == PROTOCOL_VERSION


def test_response_roundtrip_ok() -> None:
    resp = Response(ok=True, value=[1.0, 2.0, 3.0])
    parsed = parse_response(serialise_response(resp))
    assert parsed.ok
    assert parsed.value == [1.0, 2.0, 3.0]


def test_response_roundtrip_error() -> None:
    resp = Response(ok=False, error="no such robot")
    parsed = parse_response(serialise_response(resp))
    assert not parsed.ok
    assert parsed.error == "no such robot"


def test_version_compat() -> None:
    assert versions_compatible("1.0", "1.5")
    assert versions_compatible("1.99", "1.0")
    assert not versions_compatible("2.0", "1.0")


def test_remote_connection_error_when_no_server() -> None:
    rb = RemoteBackend("nobody", host="127.0.0.1", port=1, timeout=0.5)
    with pytest.raises(RemoteConnectionError):
        rb.connect()


# ── End-to-end: server + client round trip ──────────────────────────────
@pytest.fixture
def running_server():
    arms = [ArmInstance(name="arm_a", mount_pos=(0.0, 0.0, 0.10))]
    scene = _default_scene_for(arms)
    model, spec, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    home = [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
    for h in handles:
        for jname, q in zip(h.arm_joints, home, strict=True):
            data.qpos[model.jnt_qposadr[model.joint(jname).id]] = q
    mujoco.mj_forward(model, data)
    endpoints = _build_endpoints(model, data, handles)
    server = SimServer(model=model, data=data, endpoints=endpoints,
                       host="127.0.0.1", port=9876,
                       model_path=_save_model_for_clients(spec))
    server._start_tcp_listener()
    yield server
    server.shutdown()
    time.sleep(0.05)


def test_client_reads_state(running_server) -> None:
    client = RemoteBackend("arm_a", port=9876)
    try:
        client.connect()
        assert client.n_arm_joints == 6
        q = client.joint_positions
        assert q.shape == (6,)
        tcp = client.end_effector_pose()
        assert tcp.shape == (3,)
        assert client.state == "operational"
    finally:
        client.disconnect()


def test_client_sends_joint_targets(running_server) -> None:
    client = RemoteBackend("arm_a", port=9876)
    try:
        client.connect()
        client.send_joint_targets([0.1, -0.4, 1.0, 0.0, 0.0, 0.0])
        # Step physics on the server side a bit and confirm the joints moved.
        with running_server._state_lock:
            for _ in range(300):
                mujoco.mj_step(running_server.model, running_server.data)
        q = client.joint_positions
        assert abs(q[0] - 0.1) < 0.05
    finally:
        client.disconnect()


def test_unknown_robot_raises(running_server) -> None:
    client = RemoteBackend("does_not_exist", port=9876)
    with pytest.raises(RuntimeError, match="unknown robot"):
        client.connect()


# ── MotorMapper ─────────────────────────────────────────────────────────
def test_motor_mapper_identity_when_no_calibration() -> None:
    from loophole_arm.control.motor_mapper import MotorMapper

    m = MotorMapper.feetech_default()
    q = np.array([0.0, -0.5, 1.0, 0.0, 0.0, 0.0])
    counts = m.radians_to_counts(q)
    back = m.counts_to_radians(counts)
    # Default offset=0, sign=+1, so round-trip equals input (within encoder quantisation).
    assert np.allclose(back, q, atol=2e-3)


def test_motor_mapper_offset_applied() -> None:
    from loophole_arm.control.motor_mapper import MotorCalibration, MotorMapper

    # One joint with a 0.5 rad offset: kinematic 0.5 rad ↔ servo zero count.
    cal = MotorCalibration(name="J", motor_name="m", offset_rad=0.5)
    m = MotorMapper([cal])
    counts = m.radians_to_counts(np.array([0.5]))
    assert counts[0] == 2048   # zero_count
    back = m.counts_to_radians(np.array([2048]))
    assert abs(back[0] - 0.5) < 1e-6


def test_motor_mapper_sign_reversal() -> None:
    from loophole_arm.control.motor_mapper import MotorCalibration, MotorMapper

    cal = MotorCalibration(name="J", motor_name="m", sign=-1)
    m = MotorMapper([cal])
    # +0.5 rad kinematic should map to -counts/rad * 0.5 below zero.
    counts = m.radians_to_counts(np.array([0.5]))
    assert counts[0] < 2048
    back = m.counts_to_radians(counts)
    assert abs(back[0] - 0.5) < 2e-3


def test_motor_mapper_velocity_clamp() -> None:
    from loophole_arm.control.motor_mapper import MotorCalibration, MotorMapper

    cal = MotorCalibration(name="J", motor_name="m", max_count_per_step=100)
    m = MotorMapper([cal])
    clamped = m.clamp_velocity(
        target_counts=np.array([3000], dtype=np.int32),
        current_counts=np.array([2000], dtype=np.int32),
    )
    assert clamped[0] == 2100   # capped to 100-step
