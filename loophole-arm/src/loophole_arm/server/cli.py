"""``loophole-armd`` — the simulation server CLI.

Boots the simulation, registers one or more named robots, opens the viewer
(if a display is available), and listens for client connections.

  loophole-armd                          # single 'arm' on the default scene
  loophole-armd --arm arm_a --arm arm_b  # two independent arms, side by side
  loophole-armd --port 8765
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import tempfile
from pathlib import Path

import mujoco

from loophole_arm._logging import configure_logging
from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.control.limits import SafetyLimits
from loophole_arm.control.safety import SafetyBackend
from loophole_arm.control.scene import Scene
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import (
    ArmHandle,
    ArmInstance,
    build_multi_arm_model,
)
from loophole_arm.server.sim_server import RobotEndpoint, SimServer

logger = logging.getLogger("loophole_arm.server")


_FEETECH_HOME = [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
# How far apart arms sit along X when more than one is requested. 0.55 m gives
# clear visual separation without their workspaces overlapping at the home pose.
_ARM_SPACING_X = 0.55


def _default_scene_for(arms: list[ArmInstance]) -> Scene:
    """Pitch-quality default: one workbench under each arm, with the grid + axes.

    Objects only sit on the first arm's table; with more arms the user can edit
    this function or build a custom scene.
    """
    scene = Scene(
        reference_axes=True,
        reference_axes_origin=(arms[0].mount_pos[0], arms[0].mount_pos[1], arms[0].mount_pos[2]),
        table_grid=True,
    )
    for inst in arms:
        x, y, _ = inst.mount_pos
        scene.add_table(size=(0.35, 0.45), height=0.10, pos=(x, y))
    # Two cubes on the first arm's table, the canonical pick-and-place demo.
    x0, y0, _ = arms[0].mount_pos
    scene.add_object("cube", size=0.025, pos=(x0 + 0.18, y0 + 0.08, 0.13), color="orange")
    scene.add_object("cube", size=0.025, pos=(x0 + 0.18, y0 - 0.08, 0.13), color="blue")
    return scene


def _build_endpoints(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    handles: list[ArmHandle],
    per_arm_limits: dict[str, SafetyLimits] | None = None,
) -> dict[str, RobotEndpoint]:
    """Wrap each prefixed arm with its own SimBackend + SafetyBackend + IK solver.

    Each endpoint points at the handle's already-prefixed names (joints,
    gripper actuator, TCP site). Two endpoints share the same ``model``/``data``
    but address disjoint joints, so commands to one arm do not affect the other.

    ``per_arm_limits`` lets each arm have its own :class:`SafetyLimits`. Arms
    without an entry fall back to :meth:`SafetyLimits.feetech_default`.
    """
    per_arm_limits = per_arm_limits or {}
    endpoints: dict[str, RobotEndpoint] = {}
    for h in handles:
        sim = SimBackend(
            model=model, data=data,
            arm_joint_names=h.arm_joints,
            gripper_actuator=h.gripper_actuator,
            tcp_site=h.tcp_site,
        )
        limits = per_arm_limits.get(h.name, SafetyLimits.feetech_default())
        backend = SafetyBackend(sim, limits)
        backend.connect()
        backend.enable()
        solver = TCPSolver(model, h.tcp_site, arm_joint_names=h.arm_joints)
        endpoints[h.name] = RobotEndpoint(name=h.name, backend=backend, solver=solver)
    return endpoints


def _save_model_for_clients(spec: mujoco.MjSpec) -> str:
    """Save the spec + mesh assets so a client can load the same model locally.

    MjSpec.attach strips any meshdir prefix from mesh ``file`` attributes, so
    the resulting XML references each STL by bare filename (e.g.
    ``file="Link_0.STL"``). We therefore drop the meshes flat next to the XML
    rather than under a ``meshes/`` subdir.
    """
    import shutil
    out_dir = Path(tempfile.gettempdir()) / "loophole_armd"
    out_dir.mkdir(exist_ok=True)
    meshes_src = Path(__file__).resolve().parent.parent.parent.parent / "assets" / "feetech_arm" / "meshes"
    if meshes_src.exists():
        for stl in meshes_src.glob("*.STL"):
            shutil.copy2(stl, out_dir / stl.name)
    xml_path = out_dir / "model.xml"
    xml_path.write_text(spec.to_xml())
    return str(xml_path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="loophole-armd",
                                     description="Loophole sim server (multi-terminal control).")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--arm", action="append", default=None,
                        help="Name a robot endpoint (repeatable). "
                             "Ignored when --scene is given. Default: 'arm'.")
    parser.add_argument("--scene", type=Path, default=None,
                        help="Load arms + scene from a YAML config file. "
                             "See examples/scenes/*.yaml.")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    configure_logging(args.log_level)

    if os.environ.get("MUJOCO_GL") is None:
        os.environ.setdefault("MUJOCO_GL", "glfw")

    if args.scene is not None:
        # YAML path: full control over arms + scene + per-arm safety.
        from loophole_arm.server.config import load_scene_config
        scene, arms, per_arm_limits = load_scene_config(args.scene)
        logger.info("loaded scene config from %s", args.scene)
    else:
        # Default path: spread requested arm names along X, one workbench each.
        robot_names = args.arm or ["arm"]
        arms = [
            ArmInstance(name=n, mount_pos=(i * _ARM_SPACING_X, 0.0, 0.10))
            for i, n in enumerate(robot_names)
        ]
        scene = _default_scene_for(arms)
        per_arm_limits = {}        # all arms use the default
    model, spec, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    # Put every arm at its home pose so the scene opens looking like a workshop.
    for h in handles:
        for jname, q in zip(h.arm_joints, _FEETECH_HOME, strict=True):
            data.qpos[model.jnt_qposadr[model.joint(jname).id]] = q
    mujoco.mj_forward(model, data)

    endpoints = _build_endpoints(model, data, handles, per_arm_limits)
    model_path = _save_model_for_clients(spec)

    server = SimServer(
        model=model, data=data, endpoints=endpoints,
        host=args.host, port=args.port, model_path=model_path,
    )
    logger.info("starting server on %s:%d with robots: %s",
                args.host, args.port, sorted(endpoints))
    server.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
