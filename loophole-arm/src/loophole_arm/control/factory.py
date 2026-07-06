"""Assembly: wire a workcell scene + backend + IK solver + controller.

This is generic glue. It knows how to build the robot system but nothing about
any particular task. The command file calls :func:`make_sim_robot` (validate)
or :func:`make_hardware_robot` (deploy) to get a ready-to-drive
:class:`RobotController`, then issues the same commands to either.
"""
from __future__ import annotations

import mujoco

from loophole_arm.control.controller import RobotController
from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.control.limits import SafetyLimits
from loophole_arm.control.safety import SafetyBackend
from loophole_arm.control.scene import Scene
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import (
    TCP_SITE,
    WorkcellConfig,
    build_workcell_from_scene,
    build_workcell_model,
)
from loophole_arm.robots import RobotNotFoundError, available_robots, load_robot


def make_sim_robot(
    arm: str = "feetech",
    control_hz: float = 20.0,
    workcell: WorkcellConfig | None = None,
    scene: Scene | None = None,
    safety: bool = True,
    limits: SafetyLimits | None = None,
) -> tuple[RobotController, mujoco.MjModel, mujoco.MjData, list[float]]:
    """Build a SIM-backed robot ready for the command file to drive.

    Wires the industrial workcell scene, a MuJoCo sim backend, a `mink` IK
    solver targeting the TCP site, an optional safety supervisor, and the
    layered controller. This is the validation path.

    Parameters
    ----------
    workcell:
        Simple single-arm config. Builds a scene with one table.
    scene:
        Custom :class:`Scene` (composable: many tables, objects, axes, grid).
        If provided, overrides ``workcell``. The arm is mounted on top of
        ``scene.tables[0]`` if any tables exist, else at the origin.
    safety:
        Wrap the backend in a :class:`SafetyBackend` that enforces joint,
        velocity, and workspace limits. Default True. Set False only for the
        reward-hacking experiments, which deliberately explore unsafe motion.
    limits:
        Custom safety envelope; defaults to the Feetech conservative profile.

    Returns
    -------
    (controller, model, data, home_pose)
    """
    try:
        rspec = load_robot(arm)
    except RobotNotFoundError:
        raise ValueError(f"unknown arm {arm!r}; known: {available_robots()}") from None

    if scene is not None:
        # Composable path: scene + arm mounted on first table (or floor).
        mount_z = scene.tables[0].height if scene.tables else 0.0
        model = build_workcell_from_scene(
            scene, arm=arm, arm_mount_pos=(0.0, 0.0, mount_z),
        )
    else:
        cfg = workcell or WorkcellConfig(arm=arm)
        model = build_workcell_model(cfg)
    data = mujoco.MjData(model)

    home = list(rspec.home)
    data.qpos[: len(home)] = home
    mujoco.mj_forward(model, data)

    backend: SimBackend | SafetyBackend = SimBackend(
        model=model,
        data=data,
        arm_joint_names=list(rspec.joints),
        gripper_actuator=rspec.gripper_actuator,
        tcp_site=TCP_SITE,
    )
    backend.connect()
    if safety:
        backend = SafetyBackend(backend, limits or SafetyLimits.feetech_default())
        backend.enable()
    solver = TCPSolver(model, TCP_SITE, arm_joint_names=list(rspec.joints))
    controller = RobotController(backend=backend, solver=solver, control_hz=control_hz,
                                 home_pose=rspec.home)
    return controller, model, data, home


def make_hardware_robot(
    arm: str = "feetech",
    port: str = "/dev/ttyUSB0",
    control_hz: float = 20.0,
    safety: bool = True,
    limits: SafetyLimits | None = None,
) -> tuple[RobotController, list[float]]:
    """Build a HARDWARE-backed robot driving the real Feetech arm.

    Same controller and IK solver as :func:`make_sim_robot`; only the backend
    differs. Any behaviour validated in sim runs here unchanged. Safety is on
    by default and strongly recommended on hardware.

    The MuJoCo model is built purely for kinematics (FK/IK) — no dynamics are
    simulated. Joint state comes from the servo encoders.

    Returns
    -------
    (controller, home_pose)
    """
    try:
        rspec = load_robot(arm)
    except RobotNotFoundError:
        raise ValueError(f"unknown arm {arm!r}; known: {available_robots()}") from None
    if not rspec.motors:
        raise ValueError(f"arm {arm!r} has no hardware mapping; sim-only")

    # Import here so sim users never need the hardware deps installed.
    from loophole_arm.control.hardware_backend import HardwareBackend

    model = build_workcell_model(WorkcellConfig(arm=arm))
    backend: HardwareBackend | SafetyBackend = HardwareBackend(
        model=model,
        arm_joint_names=list(rspec.joints),
        lerobot_motor_names=list(rspec.motors),
        tcp_site=TCP_SITE,
        port=port,
        control_hz=control_hz,
    )
    if safety:
        backend = SafetyBackend(backend, limits or SafetyLimits.feetech_default())
    solver = TCPSolver(model, TCP_SITE, arm_joint_names=list(rspec.joints))
    controller = RobotController(backend=backend, solver=solver, control_hz=control_hz,
                                 home_pose=rspec.home)
    return controller, list(rspec.home)
