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
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import TCP_SITE, WorkcellConfig, build_workcell_model

# Per-arm wiring. ``arm_joints`` are the MuJoCo joint names (canonical order);
# ``lerobot_motors`` are the matching LeRobot motor channels for hardware. The
# end-effector control frame is the TCP site (shared by name across arms). Add
# new arms here; the command file never changes.
_ARM_WIRING: dict[str, dict] = {
    "feetech": {
        "arm_joints": ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"],
        "lerobot_motors": [
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "wrist_yaw",
        ],
        "gripper": "Joint_Gripper",
        "home": [0.0, -0.5, 1.0, 0.0, 0.0, 0.0],
    },
    "ur5e": {
        "arm_joints": ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"],
        "lerobot_motors": [],  # UR5e is a benchmark reference, not a deploy target
        "gripper": "gripper_fingers_actuator",
        "home": [0.0, -1.2, 1.6, -1.8, -1.57, 0.0],
    },
}


def make_sim_robot(
    arm: str = "feetech",
    control_hz: float = 20.0,
    workcell: WorkcellConfig | None = None,
    safety: bool = True,
    limits: SafetyLimits | None = None,
) -> tuple[RobotController, mujoco.MjModel, mujoco.MjData, list[float]]:
    """Build a SIM-backed robot ready for the command file to drive.

    Wires the industrial workcell scene, a MuJoCo sim backend, a `mink` IK
    solver targeting the TCP site, an optional safety supervisor, and the
    layered controller. This is the validation path.

    Parameters
    ----------
    safety:
        Wrap the backend in a :class:`SafetyBackend` that enforces joint,
        velocity, and workspace limits. Default True. Set False only for the
        reward-hacking experiments, which deliberately explore unsafe motion.
    limits:
        Custom safety envelope; defaults to the Feetech conservative profile.

    Returns
    -------
    (controller, model, data, home_pose)
        ``controller`` is the API the command file uses; ``model``/``data``
        are exposed for rendering or viewer attachment; ``home_pose`` is the
        arm's rest configuration.
    """
    if arm not in _ARM_WIRING:
        raise ValueError(f"unknown arm {arm!r}; known: {sorted(_ARM_WIRING)}")
    wiring = _ARM_WIRING[arm]

    cfg = workcell or WorkcellConfig(arm=arm)
    model = build_workcell_model(cfg)
    data = mujoco.MjData(model)

    home = wiring["home"]
    data.qpos[: len(home)] = home
    mujoco.mj_forward(model, data)

    backend: SimBackend | SafetyBackend = SimBackend(
        model=model,
        data=data,
        arm_joint_names=wiring["arm_joints"],
        gripper_actuator=wiring["gripper"],
        tcp_site=TCP_SITE,
    )
    backend.connect()
    if safety:
        backend = SafetyBackend(backend, limits or SafetyLimits.feetech_default())
        backend.enable()
    solver = TCPSolver(model, TCP_SITE, arm_joint_names=wiring["arm_joints"])
    controller = RobotController(backend=backend, solver=solver, control_hz=control_hz)
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
    if arm not in _ARM_WIRING:
        raise ValueError(f"unknown arm {arm!r}; known: {sorted(_ARM_WIRING)}")
    wiring = _ARM_WIRING[arm]
    if not wiring["lerobot_motors"]:
        raise ValueError(f"arm {arm!r} has no hardware mapping; sim-only")

    # Import here so sim users never need the hardware deps installed.
    from loophole_arm.control.hardware_backend import HardwareBackend

    model = build_workcell_model(WorkcellConfig(arm=arm))
    backend: HardwareBackend | SafetyBackend = HardwareBackend(
        model=model,
        arm_joint_names=wiring["arm_joints"],
        lerobot_motor_names=wiring["lerobot_motors"],
        tcp_site=TCP_SITE,
        port=port,
        control_hz=control_hz,
    )
    if safety:
        backend = SafetyBackend(backend, limits or SafetyLimits.feetech_default())
    solver = TCPSolver(model, TCP_SITE, arm_joint_names=wiring["arm_joints"])
    controller = RobotController(backend=backend, solver=solver, control_hz=control_hz)
    return controller, wiring["home"]
