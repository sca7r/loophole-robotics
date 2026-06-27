"""Layered robot control: sim/hardware backends + IK + controller + workcell.

The command file imports from here. Task logic lives in the command files, NOT
in this package — keeping the deployable command layer separate from the stable
library.

Sim-to-real: both :class:`SimBackend` and :class:`HardwareBackend` implement
:class:`RobotInterface`. Behaviours validated against the sim backend deploy to
hardware by swapping the backend (``make_sim_robot`` -> ``make_hardware_robot``);
the command code does not change.
"""
from loophole_arm.control.controller import RobotController
from loophole_arm.control.factory import make_hardware_robot, make_sim_robot
from loophole_arm.control.interface import RobotInterface
from loophole_arm.control.kinematics import IKSolution, TCPSolver
from loophole_arm.control.limits import (
    SafetyLimits,
    SafetyState,
    SafetyViolation,
)
from loophole_arm.control.safety import SafetyBackend
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import TCP_SITE, WorkcellConfig, build_workcell_model

__all__ = [
    "TCP_SITE",
    "IKSolution",
    "RobotController",
    "RobotInterface",
    "SafetyBackend",
    "SafetyLimits",
    "SafetyState",
    "SafetyViolation",
    "SimBackend",
    "TCPSolver",
    "WorkcellConfig",
    "build_workcell_model",
    "make_hardware_robot",
    "make_sim_robot",
]
