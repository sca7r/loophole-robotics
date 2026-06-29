"""``loophole-arm-teleop`` — keyboard teleoperation client.

Drive a robot's TCP with the numpad. Live coordinate display, IK-driven motion,
the same safety layer the rest of the stack runs on. Connects to a running
``loophole-armd`` server.

Key layout (numpad):

      7  8  9          7 = +Z up        8 = +Y left (away)   9 = open gripper
      4  5  6          4 = -X back      5 = home            6 = +X forward
      1  2  3          1 = close grip   2 = -Y right        3 = -Z down

      +  -             +  =  larger step size               -  =  smaller step size
      0 (or .)         print current TCP coordinates

      Esc / q          quit
      e                e-stop
      r                reset safety
"""
from __future__ import annotations

import argparse
import logging
import sys

from loophole_arm._logging import configure_logging
from loophole_arm.control.controller import RobotController
from loophole_arm.control.kinematics import TCPSolver
from loophole_arm.control.workcell import TCP_SITE
from loophole_arm.server.remote_backend import RemoteBackend

logger = logging.getLogger("loophole_arm.teleop")


# Default per-keypress step sizes (metres for XYZ, gripper fraction).
_STEPS = {
    "xy": 0.020,    # 2 cm per press — feels right for picking
    "z": 0.020,     # 2 cm per press
}
_STEP_SCALES = [0.005, 0.010, 0.020, 0.040, 0.080]   # cycle with + / -


_HELP = """
Numpad teleop:
  7 +Z   8 +Y   9 open
  4 -X   5 home 6 +X
  1 close 2 -Y  3 -Z
  + / -  bigger/smaller step
  0 / .  print position
  e      e-stop      r  reset safety
  q / Esc  quit
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="loophole-arm-teleop",
                                     description="Numpad teleop for a Loophole arm.")
    parser.add_argument("robot", help="Name of the robot endpoint on the server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    configure_logging(args.log_level)

    # Connect
    print(f"Connecting to {args.host}:{args.port} as {args.robot!r}...")
    backend = RemoteBackend(args.robot, host=args.host, port=args.port)
    backend.connect()
    print(f"Connected. n_arm_joints = {backend.n_arm_joints}")

    # Build the local IK solver against the kinematic model the server sent.
    arm_joint_names = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
    try:
        model = backend.kinematic_model()
    except RuntimeError:
        print("ERROR: server did not send a kinematic model; cannot run IK teleop.")
        return 2
    solver = TCPSolver(model, TCP_SITE, arm_joint_names=arm_joint_names)
    controller = RobotController(backend=backend, solver=solver, control_hz=20.0)
    controller.enable()

    # Home pose for the '5' key.
    home_pose = [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]

    print(_HELP)
    step_idx = 2
    gripper_closed = False

    try:
        with _terminal_raw():
            while True:
                tcp = backend.end_effector_pose()
                step = _STEP_SCALES[step_idx]
                sys.stdout.write(
                    f"\rTCP  x={tcp[0]:+.3f}  y={tcp[1]:+.3f}  z={tcp[2]:+.3f}  "
                    f"step={step*1000:3.0f}mm  grip={'CLOSED' if gripper_closed else 'open'}   "
                )
                sys.stdout.flush()

                key = _getch(timeout=0.5)
                if key is None:
                    continue

                if key in ("q", "\x1b"):  # q or Esc
                    break
                if key == "e":
                    print("\n[E-STOP]")
                    controller.estop()
                    continue
                if key == "r":
                    print("\n[reset + enable]")
                    controller.reset_safety()
                    controller.enable()
                    continue
                if key == "+":
                    step_idx = min(step_idx + 1, len(_STEP_SCALES) - 1)
                    continue
                if key == "-":
                    step_idx = max(step_idx - 1, 0)
                    continue
                if key in ("0", "."):
                    print(f"\nTCP @ ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f})  m")
                    continue
                if key == "5":   # home
                    controller.move_joints(home_pose, duration=1.0)
                    continue
                if key == "9":
                    controller.open_gripper()
                    gripper_closed = False
                    continue
                if key == "1":
                    controller.close_gripper()
                    gripper_closed = True
                    continue

                # XYZ jog keys — compute new target and run move_to.
                dx = dy = dz = 0.0
                if key == "6":
                    dx = +step
                elif key == "4":
                    dx = -step
                elif key == "8":
                    dy = +step
                elif key == "2":
                    dy = -step
                elif key == "7":
                    dz = +step
                elif key == "3":
                    dz = -step
                else:
                    continue

                target = (float(tcp[0] + dx), float(tcp[1] + dy), float(tcp[2] + dz))
                ok = controller.move_to(*target, duration=0.25)
                if not ok:
                    sys.stdout.write("\n  (rejected — outside envelope or unreachable)\n")
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDisconnecting…")
        with contextlib.suppress(Exception):
            backend.disconnect()
    return 0


# ── Cross-platform single-key input (no extra deps) ─────────────────────
import contextlib  # noqa: E402


@contextlib.contextmanager
def _terminal_raw():
    """Put the terminal in raw mode so we read keys without waiting for Enter."""
    import sys
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _getch(timeout: float = 0.5) -> str | None:
    """Read one keypress with a timeout. Returns ``None`` on no input."""
    import select
    import sys
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    return ch


if __name__ == "__main__":
    sys.exit(main())
