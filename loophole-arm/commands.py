"""
╔══════════════════════════════════════════════════════════════════════════╗
║  COMMANDS — this is the ONLY file you edit to change what the robot does. ║
╚══════════════════════════════════════════════════════════════════════════╝

Everything else (the arm model, the scene, the physics, the IK, the gripper
logic) is generic and lives in the library. To make the robot perform a
different task, you only change the `run()` function below.

When you deploy to real hardware, this file does not change — you swap the
backend (sim → hardware) in one line, and these same commands drive the
physical arm.

──────────────────────────────────────────────────────────────────────────
THREE LAYERS OF CONTROL  (use whichever fits the task)
──────────────────────────────────────────────────────────────────────────

  Layer 1 — JOINT SPACE      robot.move_joints([j1, j2, j3, j4, j5, j6])
                             Direct joint angles in radians. Full control,
                             lowest level.

  Layer 2 — TASK SPACE       robot.move_to(x, y, z)
                             Move the gripper to a Cartesian point. The IK
                             solver figures out the joint angles.

  Layer 3 — SKILLS           Pick(x, y, z).run(robot)
                             Place(x, y, z).run(robot)
                             robot.home(HOME_POSE)
                             robot.open_gripper() / robot.close_gripper()
                             High-level semantic actions.

──────────────────────────────────────────────────────────────────────────
RUN IT
──────────────────────────────────────────────────────────────────────────

  # Validate in simulation — live viewer (on a machine with a display):
  python commands.py

  # Headless render to MP4 (works anywhere):
  python commands.py --render out.mp4

  # Deploy the SAME commands to the real arm (one flag — run() is unchanged):
  python commands.py --hardware --port /dev/ttyUSB0
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Make the package importable when running directly from the repo.
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from loophole_arm.control import make_sim_robot

# ── Configuration ────────────────────────────────────────────────────────
ARM = "feetech"          # "feetech" (your arm) or "ur5e" (industrial reference)


# ══════════════════════════════════════════════════════════════════════════
#  YOUR TASK — edit this function
# ══════════════════════════════════════════════════════════════════════════
def run(robot, home_pose) -> None:
    """Define the robot's behaviour here.

    `robot`      — the controller (all three layers of commands)
    `home_pose`  — the arm's rest configuration, for returning home

    The example below is a simple pick-and-place demo. Replace it with your
    own sequence of commands.
    """
    # Start from a known pose.
    robot.home(home_pose)

    # ── Layer 3 (skills): a clean pick-and-place ───────────────────────
    # Coordinates are in metres, relative to the arm base.
    # The work surface is at z ≈ 0.10 (the table top).

    pick_xy = (0.18, 0.08)      # where to pick from
    place_xy = (0.18, -0.08)    # where to place
    surface_z = 0.12            # just above the table surface

    # Skills are the product surface: parameterised, composable primitives.
    # The same skill objects run unchanged against sim, remote, or hardware.
    from loophole_arm.skills import Pick, Place
    from loophole_arm.skills.engine import SkillEngine
    engine = SkillEngine()

    print("→ Picking from", pick_xy)
    engine.run(Pick(x=pick_xy[0], y=pick_xy[1], z=surface_z), robot)

    print("→ Placing at", place_xy)
    engine.run(Place(x=place_xy[0], y=place_xy[1], z=surface_z), robot)

    # ── Return home ────────────────────────────────────────────────────
    print("→ Returning home")
    robot.home(home_pose)

    # ────────────────────────────────────────────────────────────────────
    # OTHER THINGS YOU CAN DO (examples — uncomment to try):
    #
    # Layer 2 — trace a square in the air with the gripper:
    #   robot.move_to(0.15,  0.10, 0.25)
    #   robot.move_to(0.15, -0.10, 0.25)
    #   robot.move_to(0.25, -0.10, 0.25)
    #   robot.move_to(0.25,  0.10, 0.25)
    #
    # Layer 1 — wave by moving a single joint:
    #   robot.move_joints([ 0.5, -0.5, 1.0, 0.0, 0.0, 0.0])
    #   robot.move_joints([-0.5, -0.5, 1.0, 0.0, 0.0, 0.0])
    #
    # Gripper test:
    #   robot.open_gripper()
    #   robot.close_gripper()
    # ────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════
#  Runner — you don't need to edit below this line
# ══════════════════════════════════════════════════════════════════════════
def main() -> int:
    parser = argparse.ArgumentParser(description="Run robot commands")
    parser.add_argument("--render", type=Path, default=None,
                        help="Headless: render to this MP4 instead of opening a window")
    parser.add_argument("--arm", default=ARM, choices=["feetech", "ur5e"])
    parser.add_argument("--hardware", action="store_true",
                        help="Run on the REAL arm instead of simulation")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port of the Feetech bus (with --hardware)")
    args = parser.parse_args()

    # ── The sim → hardware swap is THIS one decision ───────────────────────
    # The run() function above is identical either way. Validate in sim, then
    # add --hardware to deploy the same commands to the physical arm.
    if args.hardware:
        from loophole_arm.control import make_hardware_robot
        robot, home_pose = make_hardware_robot(arm=args.arm, port=args.port)
        robot.backend.connect()
        robot.enable()                 # arm the safety supervisor for motion
        try:
            run(robot, home_pose)
        finally:
            robot.estop()              # latch motion off before releasing
            robot.backend.disconnect()
        return 0

    robot, model, data, home_pose = make_sim_robot(arm=args.arm, control_hz=20.0)
    if args.render:
        _run_headless(robot, model, data, home_pose, args.render)
    else:
        _run_live(robot, model, data, home_pose)
    return 0


def _run_live(robot, model, data, home_pose) -> None:
    """Open a live MuJoCo window and run the commands in real time."""
    try:
        import mujoco.viewer
    except ImportError:
        print("mujoco.viewer unavailable — use --render out.mp4 instead.")
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.9
        viewer.cam.azimuth = 140
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.12, 0.0, 0.18]
        robot._viewer_sync = viewer.sync   # let the controller refresh the view
        run(robot, home_pose)
        print("Done. Close the window to exit.")
        while viewer.is_running():
            viewer.sync()


def _run_headless(robot, model, data, home_pose, out_path: Path) -> None:
    """Render the command sequence to an MP4 (no display needed)."""
    import os
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    import imageio
    import mujoco

    frames: list = []
    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.distance = 0.9
    cam.azimuth = 140
    cam.elevation = -20
    cam.lookat[:] = [0.12, 0.0, 0.18]

    # Capture frames by hooking the controller's sync callback.
    frame_skip = max(1, int((1 / 30) / model.opt.timestep))
    counter = {"n": 0}

    def capture() -> None:
        if counter["n"] % frame_skip == 0:
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
        counter["n"] += 1

    # Wrap backend.step so every physics step gets a chance to be captured.
    original_step = robot.backend.step

    def step_and_capture(dt: float) -> None:
        import mujoco as _mj
        n = max(1, int(dt / model.opt.timestep))
        for _ in range(n):
            _mj.mj_step(model, data)
            capture()

    robot.backend.step = step_and_capture  # type: ignore[method-assign]

    run(robot, home_pose)
    robot.backend.step = original_step  # type: ignore[method-assign]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(str(out_path), frames, fps=30, codec="libx264",
                    quality=8, macro_block_size=1)
    print(f"wrote {out_path} ({len(frames)} frames)")


if __name__ == "__main__":
    sys.exit(main())
