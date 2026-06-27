"""LeRobot-compatible configuration for the Loophole Arm.

The arm is a 6-DOF Feetech-servo manipulator with a 1-DOF parallel gripper.
Configuration follows LeRobot's :class:`RobotConfig` schema so the arm can be
driven by ``lerobot-teleoperate``, ``lerobot-record``, and ``lerobot-train``
out of the box.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("loophole_arm")
@dataclass
class LoopholeArmConfig(RobotConfig):
    """Configuration for the Loophole Arm — a Feetech-servo 6-DOF + gripper.

    Attributes:
        port: Serial port the Feetech bus is attached to (e.g. ``/dev/ttyUSB0``).
        disable_torque_on_disconnect: Disable servo torque on graceful shutdown.
            Recommended ``True`` for safety — the arm goes limp instead of
            holding its last commanded position.
        max_relative_target: Per-step relative-target clamp in joint units
            (degrees if ``use_degrees`` else normalised [-100, 100]). Acts as
            the *last* line of velocity-limit defence before the servo bus.
        cameras: Optional camera configs keyed by name. Camera streams appear
            as ``<name>`` observation entries to downstream policies.
        use_degrees: Encode joint positions in degrees. Set ``False`` for
            normalised values in [-100, 100]; downstream policies must match.
    """

    port: str = "/dev/ttyUSB0"
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = 10.0
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    use_degrees: bool = True
