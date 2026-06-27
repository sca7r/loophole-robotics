"""Loophole Arm — a LeRobot-compatible 6-DOF Feetech-servo arm with a
MuJoCo-based reward-hacking demonstration suite.

The package has two layers with deliberately decoupled imports:

1. **Simulation** (:mod:`loophole_arm.sim`, :mod:`loophole_arm.optimizer`,
   :mod:`loophole_arm.rewards`) — pure-Python on top of MuJoCo + NumPy.
   Importable without ``lerobot`` installed.

2. **Real hardware** (:class:`LoopholeArm`, :class:`LoopholeArmConfig`) —
   wraps :mod:`lerobot.motors.feetech`. Pulled in only when the consumer
   imports them; install with ``pip install loophole-arm[hardware]``.

Once installed, ``lerobot --robot.type=loophole_arm ...`` discovers the
robot automatically via the ``lerobot.robots`` entry point.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

from loophole_arm._version import __version__

if TYPE_CHECKING:  # pragma: no cover
    from loophole_arm.robot import LoopholeArm
    from loophole_arm.robot_config import LoopholeArmConfig

__all__ = ["LoopholeArm", "LoopholeArmConfig", "__version__"]


# Lazy attribute access — keeps lerobot off the import path of the sim layer.
def __getattr__(name: str) -> Any:
    if name == "LoopholeArm":
        return importlib.import_module("loophole_arm.robot").LoopholeArm
    if name == "LoopholeArmConfig":
        return importlib.import_module("loophole_arm.robot_config").LoopholeArmConfig
    raise AttributeError(f"module 'loophole_arm' has no attribute {name!r}")
