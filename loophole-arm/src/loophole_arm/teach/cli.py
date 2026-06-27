"""``loophole-arm-teach`` — teach-and-repeat command line.

Subcommands
-----------
    teach        Interactively teach a skill by setting waypoints (live viewer).
    play         Replay a saved skill (live viewer, or --render to MP4).
    show         Print the waypoints of a saved skill.
    demo         Build + save an example pick-and-place skill, then replay it.

All of this runs in simulation — no hardware required. The skills you save here
replay on the physical arm later, unchanged.
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from loophole_arm._logging import configure_logging
from loophole_arm.control import make_sim_robot
from loophole_arm.teach.player import TrajectoryPlayer
from loophole_arm.teach.session import TeachSession
from loophole_arm.teach.trajectory import Trajectory

logger = logging.getLogger("loophole_arm.teach")

_INTERACTIVE_HELP = """
Teaching commands (type and press enter):
  cart X Y Z [label]   move TCP to (x,y,z) metres and record the waypoint
  joints J1..J6 [label] move to absolute joint angles (radians) and record
  grip open|close      actuate and record the gripper
  dwell SECONDS        record a pause
  home                 move to the home pose and record it
  undo                 remove the last recorded waypoint
  list                 show waypoints recorded so far
  save NAME            save the skill to skills/NAME.json
  help                 show this help
  done                 finish (saves if a name was given) and exit
"""


# ── teach (interactive) ──────────────────────────────────────────────────
def cmd_teach(args: argparse.Namespace) -> int:
    robot, model, data, home = make_sim_robot(arm=args.arm)
    session = TeachSession(robot, name=args.name or "untitled", arm=args.arm)

    viewer_ctx = _maybe_viewer(model, data, robot)
    print(f"\nTeaching '{session.trajectory.name}' on the {args.arm} arm.")
    print(_INTERACTIVE_HELP)

    with viewer_ctx:
        robot.home(home)
        saved_to: str | None = None
        while True:
            try:
                line = input("teach> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd == "done":
                if args.name and saved_to is None:
                    saved_to = session.save(f"skills/{args.name}.json")
                break
            elif cmd == "help":
                print(_INTERACTIVE_HELP)
            elif cmd == "home":
                session.teach_joints(home, label="home")
            elif cmd == "cart" and len(parts) >= 4:
                x, y, z = (float(parts[1]), float(parts[2]), float(parts[3]))
                label = " ".join(parts[4:])
                if not session.teach_cartesian(x, y, z, label=label):
                    print("  (unreachable or outside safety envelope — not recorded)")
            elif cmd == "joints" and len(parts) >= 7:
                j = [float(p) for p in parts[1:7]]
                session.teach_joints(j, label=" ".join(parts[7:]))
            elif cmd == "grip" and len(parts) >= 2:
                session.teach_gripper(1.0 if parts[1] == "close" else 0.0, label=" ".join(parts[2:]))
            elif cmd == "dwell" and len(parts) >= 2:
                session.teach_dwell(float(parts[1]), label=" ".join(parts[2:]))
            elif cmd == "undo":
                if session.trajectory.waypoints:
                    session.trajectory.waypoints.pop()
                    print(f"  removed last waypoint ({len(session.trajectory)} remain)")
            elif cmd == "list":
                _print_waypoints(session.trajectory)
            elif cmd == "save" and len(parts) >= 2:
                saved_to = session.save(f"skills/{parts[1]}.json")
                print(f"  saved → {saved_to}")
            else:
                print("  ? type 'help' for commands")

    if saved_to:
        print(f"\nSaved skill to {saved_to}")
        print(f"Replay it with:  loophole-arm-teach play {saved_to}")
    return 0


# ── play ─────────────────────────────────────────────────────────────────
def cmd_play(args: argparse.Namespace) -> int:
    traj = Trajectory.load(args.skill)
    robot, model, data, home = make_sim_robot(arm=traj.arm)
    robot.home(home)
    player = TrajectoryPlayer(robot)

    if args.render:
        _play_headless(robot, model, data, traj, player, args.render, args.loops)
    else:
        with _maybe_viewer(model, data, robot):
            ok = player.play(traj, loops=args.loops)
            print("Playback complete." if ok else "Playback stopped early.")
    return 0


# ── show ─────────────────────────────────────────────────────────────────
def cmd_show(args: argparse.Namespace) -> int:
    traj = Trajectory.load(args.skill)
    print(f"\nSkill: {traj.name}")
    print(f"Arm:   {traj.arm}   control_hz: {traj.control_hz}   created: {traj.created}")
    _print_waypoints(traj)
    return 0


# ── demo ───────────────────────────────────────────────────────────────────
def cmd_demo(args: argparse.Namespace) -> int:
    """Build the canonical pick-and-place skill, save it, then replay it."""
    robot, model, data, home = make_sim_robot(arm=args.arm)
    session = TeachSession(robot, name="pick_place_demo", arm=args.arm)

    print("\n=== TEACH: building a pick-and-place skill by waypoints ===")
    robot.home(home)
    session.teach_joints(home, label="home")
    session.teach_cartesian(0.18, 0.08, 0.18, label="above pick")
    session.teach_cartesian(0.18, 0.08, 0.12, label="descend")
    session.teach_gripper(1.0, label="grasp")
    session.teach_cartesian(0.18, 0.08, 0.18, label="lift")
    session.teach_cartesian(0.18, -0.08, 0.18, label="above place")
    session.teach_cartesian(0.18, -0.08, 0.12, label="descend")
    session.teach_gripper(0.0, label="release")
    session.teach_cartesian(0.18, -0.08, 0.18, label="retract")
    session.teach_joints(home, label="home")
    path = session.save(f"skills/{session.trajectory.name}.json")
    print(f"Taught and saved {len(session.trajectory)} waypoints → {path}")

    print("\n=== REPEAT: replaying the taught skill ===")
    robot.home(home)
    player = TrajectoryPlayer(robot)
    if args.render:
        _play_headless(robot, model, data, session.trajectory, player, args.render, 1)
    else:
        with _maybe_viewer(model, data, robot):
            player.play(session.trajectory)
    print("\nDone. The same skill file will run the physical arm later, unchanged.")
    return 0


# ── helpers ────────────────────────────────────────────────────────────────
def _print_waypoints(traj: Trajectory) -> None:
    if not traj.waypoints:
        print("  (no waypoints yet)")
        return
    for i, wp in enumerate(traj.waypoints, 1):
        if wp.kind == "cartesian" and wp.position:
            payload = f"({wp.position[0]:.3f}, {wp.position[1]:.3f}, {wp.position[2]:.3f})"
        elif wp.kind == "joint" and wp.joints:
            payload = "[" + ", ".join(f"{j:.2f}" for j in wp.joints) + "]"
        elif wp.kind == "gripper":
            payload = "close" if (wp.gripper or 0) >= 0.5 else "open"
        else:
            payload = f"{wp.duration:.1f}s"
        label = f"  — {wp.label}" if wp.label else ""
        print(f"  {i:2d}. {wp.kind:9s} {payload}{label}")


def _maybe_viewer(model, data, robot):
    """Return a live-viewer context if a display is available, else a no-op."""
    import contextlib
    import os
    import platform

    # On Linux a viewer needs a display; checking up front avoids a hard GLFW
    # crash on headless machines/servers. macOS/Windows always have one.
    if platform.system() == "Linux" and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        logger.info("no display detected — running without live viewer "
                    "(use --render to capture an MP4)")
        return contextlib.nullcontext()

    try:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    except Exception:
        return contextlib.nullcontext()

    viewer.cam.distance = 0.9
    viewer.cam.azimuth = 140
    viewer.cam.elevation = -20
    viewer.cam.lookat[:] = [0.12, 0.0, 0.18]
    robot._viewer_sync = viewer.sync
    return viewer


def _play_headless(robot, model, data, traj, player, out_path: Path, loops: int) -> None:
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
    every = max(1, int((1 / 30) / model.opt.timestep))
    counter = {"n": 0}

    orig_step = robot.backend.step

    def step_and_capture(dt: float) -> None:
        n = max(1, int(dt / model.opt.timestep))
        for _ in range(n):
            mujoco.mj_step(model, data)
            if counter["n"] % every == 0:
                renderer.update_scene(data, cam)
                frames.append(renderer.render())
            counter["n"] += 1

    robot.backend.step = step_and_capture  # type: ignore[method-assign]
    player.play(traj, loops=loops)
    robot.backend.step = orig_step  # type: ignore[method-assign]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(str(out_path), frames, fps=30, codec="libx264", quality=8, macro_block_size=1)
    print(f"wrote {out_path} ({len(frames)} frames)")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="loophole-arm-teach",
                                description="Teach-and-repeat for the Loophole Arm (sim).")
    p.add_argument("--log-level", default="INFO")
    sub = p.add_subparsers(dest="cmd", required=True)

    pt = sub.add_parser("teach", help="Interactively teach a skill")
    pt.add_argument("--name", help="Skill name (saved to skills/NAME.json on 'done')")
    pt.add_argument("--arm", default="feetech", choices=["feetech", "ur5e"])
    pt.set_defaults(func=cmd_teach)

    pp = sub.add_parser("play", help="Replay a saved skill")
    pp.add_argument("skill", help="Path to a saved .json skill")
    pp.add_argument("--loops", type=int, default=1)
    pp.add_argument("--render", type=Path, help="Headless: render to this MP4")
    pp.set_defaults(func=cmd_play)

    ps = sub.add_parser("show", help="Print a skill's waypoints")
    ps.add_argument("skill")
    ps.set_defaults(func=cmd_show)

    pd = sub.add_parser("demo", help="Teach + replay an example pick-and-place")
    pd.add_argument("--arm", default="feetech", choices=["feetech", "ur5e"])
    pd.add_argument("--render", type=Path, help="Headless: render to this MP4")
    pd.set_defaults(func=cmd_demo)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_level)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
