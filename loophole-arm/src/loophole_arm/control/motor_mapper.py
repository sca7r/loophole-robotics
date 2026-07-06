"""MotorMapper — the software ↔ servo conversion bridge.

This is the structural seam between the software view of the arm (SI units,
canonical joint order, kinematic zero) and the physical motor view (encoder
counts, calibrated zero offsets, motor-native limits and signs). Building it
now keeps a single, named place to fill in real values once the hardware is on
the bench, instead of scattering ad-hoc unit conversions through the codebase.

Three things must match between sim and the real motors, and this class owns
each of them explicitly:

  1. Units — software talks in radians; servos talk in encoder counts. The
     conversion is ``count = (q - offset) * counts_per_rad * sign``.

  2. Calibration zero — every Feetech servo has a per-motor zero offset
     determined when you home it. The URDF zero is the kinematic zero, which
     is *not* the same as the servo's encoder zero. ``offset`` captures this.

  3. Direction sign — depending on how the servo is mounted, "positive joint
     angle" may correspond to a positive or negative encoder direction.
     ``sign`` (±1) captures this per joint.

The mapper also enforces motor-native limits (max encoder count, max speed)
which sim is more forgiving about than real hardware.

Status: structural. All counts/offsets/signs default to placeholders. They get
filled in during the calibration session on the real arm (TODO(hardware)).
The structure is correct now so the wiring downstream does not change.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class MotorCalibration:
    """Per-motor calibration. One :class:`MotorCalibration` per joint.

    Attributes
    ----------
    name:
        Joint name (matches the URDF/MJCF). Identifies which DoF this is.
    motor_name:
        LeRobot motor channel name (e.g. ``"shoulder_pan"``).
    offset_rad:
        Calibration offset: the URDF-kinematic angle (radians) at which the
        servo's encoder reads ``zero_count``. Measured during homing on the
        physical arm.
    zero_count:
        The encoder count corresponding to ``offset_rad``. Feetech servos use
        12-bit absolute encoders (0..4095), with zero typically the midpoint.
    counts_per_rad:
        Encoder counts per radian. For a Feetech STS3215 with 12-bit absolute
        encoding over 360°, this is ``4096 / (2π) ≈ 651.74``.
    sign:
        ``+1`` if positive-radians means positive-counts, ``-1`` if reversed.
        Determined empirically by jogging the joint on the bench.
    min_count, max_count:
        Motor-native safe range. Stricter than the URDF joint limit, because
        servos can stall against mechanical hard-stops a few counts before
        their full range. Hardware-only safety; sim ignores these.
    max_count_per_step:
        Velocity limit, expressed in counts-per-control-tick. The servo will
        clip motion faster than this and may report a stall.
    """
    name: str
    motor_name: str
    offset_rad: float = 0.0
    zero_count: int = 2048
    counts_per_rad: float = 4096.0 / (2.0 * math.pi)
    sign: int = 1
    min_count: int = 0
    max_count: int = 4095
    max_count_per_step: int = 200


@dataclass
class MotorMapper:
    """Convert between software (radians) and motor (encoder counts) spaces.

    Built from a list of per-motor calibrations, one per arm joint, in
    canonical order. Use it whenever crossing the software/hardware seam —
    nowhere else.

    The default Feetech-arm calibration produced by :meth:`feetech_default`
    is a *placeholder* (zero offsets, midpoint zero counts). It is correct
    structurally and lets the bridge compile and be tested in sim. The real
    values need to be measured on the bench during arm bring-up; the
    ``TODO(hardware)`` markers in :class:`HardwareBackend` reference these.
    """
    calibrations: list[MotorCalibration]
    _by_motor: dict[str, MotorCalibration] = field(default_factory=dict, repr=False)

    def __post_init__(self) -> None:
        self._by_motor = {c.motor_name: c for c in self.calibrations}

    # ── Construction ────────────────────────────────────────────────────
    @staticmethod
    def feetech_default() -> MotorMapper:
        """Structural placeholder calibration for the Feetech arm.

        All offsets are zero and all signs are +1. These need to be measured
        during the first hardware bring-up. The motor-name → joint mapping is
        the real one already used by :class:`HardwareBackend`.
        """
        from loophole_arm.robots import load_robot
        rspec = load_robot("feetech")
        joints = list(rspec.joints)
        motors = list(rspec.motors)
        return MotorMapper([
            MotorCalibration(name=j, motor_name=m) for j, m in zip(joints, motors, strict=True)
        ])

    # ── Conversions ─────────────────────────────────────────────────────
    def radians_to_counts(self, q: NDArray[np.float64]) -> NDArray[np.int32]:
        """Software joint angles (rad) → servo encoder counts."""
        if len(q) != len(self.calibrations):
            raise ValueError(f"expected {len(self.calibrations)} joint angles, got {len(q)}")
        out = np.empty(len(q), dtype=np.int32)
        for i, (val, cal) in enumerate(zip(q, self.calibrations, strict=True)):
            raw = cal.zero_count + cal.sign * (val - cal.offset_rad) * cal.counts_per_rad
            out[i] = int(np.clip(round(raw), cal.min_count, cal.max_count))
        return out

    def counts_to_radians(self, counts: NDArray[np.int32]) -> NDArray[np.float64]:
        """Servo encoder counts → software joint angles (rad)."""
        if len(counts) != len(self.calibrations):
            raise ValueError(f"expected {len(self.calibrations)} counts, got {len(counts)}")
        out = np.empty(len(counts), dtype=np.float64)
        for i, (c, cal) in enumerate(zip(counts, self.calibrations, strict=True)):
            out[i] = cal.offset_rad + cal.sign * (c - cal.zero_count) / cal.counts_per_rad
        return out

    def clamp_velocity(
        self,
        target_counts: NDArray[np.int32],
        current_counts: NDArray[np.int32],
    ) -> NDArray[np.int32]:
        """Apply the motor-native per-tick velocity cap.

        Hardware-only: a command faster than the servo can physically execute
        in one control tick is clipped to the servo's max step. This is a
        stricter, motor-realistic version of our software velocity limit.
        """
        out = np.empty_like(target_counts)
        for i, (tgt, cur, cal) in enumerate(zip(target_counts, current_counts, self.calibrations, strict=True)):
            step = int(np.clip(tgt - cur, -cal.max_count_per_step, cal.max_count_per_step))
            out[i] = int(cur) + step
        return out
