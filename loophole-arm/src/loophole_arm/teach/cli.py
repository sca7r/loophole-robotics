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
╭─ Teach prompt — the industry workflow ─────────────────────────────────╮
│                                                                         │
│  Step 1 · JOG the gripper into position (no typing coordinates):        │
│    jog x+ / x- / y+ / y- / z+ / z-    nudge the TCP one step            │
│    step SIZE                          set jog step in metres            │
│                                       (default 0.02 = 2 cm)             │
│    where                              print current TCP + joints        │
│                                                                         │
│  Step 2 · TEACH the pose a name:                                        │
│    teach NAME            save the current pose  (e.g. teach pick_pose)  │
│    points                list all taught points                         │
│                                                                         │
│  Step 3 · RUN skills on taught names OR live objects:                   │
│    objects               list scene objects with live positions         │
│    pick NAME             NAME is a taught point or an object            │
│    place NAME            approach, descend, act, lift                   │
│    open / close          gripper directly                               │
│                                                                         │
│  RECORDING a replayable skill (same as before):                         │
│    cart X Y Z [label]    move + record waypoint                         │
│    joints J1..J6 [label] move + record joint waypoint                   │
│    grip open|close       actuate + record                               │
│    dwell SECONDS         record a pause                                 │
│    goto X Y Z            preview-move WITHOUT recording                 │
│    home | list | undo | save NAME | done                                │
│                                                                         │
│  HELP:  help (this menu)   keys (coordinate system tip)                 │
╰─────────────────────────────────────────────────────────────────────────╯

Typical session:  jog to the object → `teach pick_pose` → jog to the bin →
`teach place_pose` → `pick pick_pose` → `place place_pose`. Done.
"""

_COORDS_HELP = """
EVERY coordinate you type (cart, goto, skills) is in the BASE frame:
an absolute position measured from the arm's base on the floor. It is
never relative to the gripper. The gripper (TCP) is the point that MOVES
to your coordinate. Use `objects` or `where` to see live positions both
ways: absolute, and as "how far from the gripper".

Coordinate system (right-handed, arm base at origin):

   +X = forward (away from you, into the scene)
   +Y = left (when facing the arm from the front)
   +Z = up

Useful ranges for the Feetech arm on the workbench:
   X: 0.05 to 0.30 m    (5-30 cm forward of the arm base)
   Y: -0.25 to 0.25 m   (left/right)
   Z: 0.10 to 0.35 m    (table surface @ ~0.10 m, max reach ~0.35 m)

If the scene has reference axes turned on, you'll see them in the viewer:
   RED = +X     GREEN = +Y     BLUE = +Z
"""


# ── teach (interactive) ──────────────────────────────────────────────────
_POINTS_FILE = "workspace/points.json"
# Table-strike guard for the pick/place workflow: never command the TCP
# below 5 mm above the standard table top (0.10 m). Low-height reach itself
# works since 0.10.0 (IK branch restart + closed-loop refinement).
_GRASP_MIN_Z = 0.105


def _describe_offset(dx: float, dy: float, dz: float) -> str:
    """Human phrasing of a delta from the gripper, in centimetres.

    Example: "4.2cm forward, 1.0cm left, 2.8cm below". Signs follow the
    base-frame convention printed by the `keys` command.
    """
    parts = []
    if abs(dx) >= 0.002:
        parts.append(f"{abs(dx)*100:.1f}cm {'forward' if dx > 0 else 'back'}")
    if abs(dy) >= 0.002:
        parts.append(f"{abs(dy)*100:.1f}cm {'left' if dy > 0 else 'right'}")
    if abs(dz) >= 0.002:
        parts.append(f"{abs(dz)*100:.1f}cm {'above' if dz > 0 else 'below'}")
    return ", ".join(parts) if parts else "at the gripper"


def _interactive_loop(
    robot,
    home: list[float],
    session: TeachSession,
    save_name: str | None,
) -> str | None:
    """The interactive teach loop. Backend-agnostic: works for both the local
    ``make_sim_robot`` controller and a remote controller bound to ``loophole-armd``.

    Implements the industry teach-pendant workflow: jog into position, teach
    the pose a name, then run Pick/Place skills against taught names. Taught
    points persist to ``workspace/points.json`` across sessions.
    """
    from loophole_arm.skills import Pick, Place
    from loophole_arm.skills.engine import SkillEngine, SkillNotFoundError

    engine = SkillEngine()
    n_loaded = engine.load_points(_POINTS_FILE)
    if n_loaded:
        print(f"(loaded {n_loaded} taught points from {_POINTS_FILE}: "
              f"{', '.join(engine.point_names())})")

    jog_step = 0.02   # metres per jog command; `step SIZE` changes it
    print(_INTERACTIVE_HELP)
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

        # ── Industry workflow: jog / teach / points / pick / place ──────
        if cmd == "jog" and len(parts) >= 2:
            axes = {"x+": (jog_step, 0, 0), "x-": (-jog_step, 0, 0),
                    "y+": (0, jog_step, 0), "y-": (0, -jog_step, 0),
                    "z+": (0, 0, jog_step), "z-": (0, 0, -jog_step)}
            d = axes.get(parts[1].lower())
            if d is None:
                print("  ? jog wants one of: x+ x- y+ y- z+ z-")
                continue
            tcp = robot.backend.end_effector_pose()
            ok = robot.move_to(float(tcp[0] + d[0]), float(tcp[1] + d[1]),
                               float(tcp[2] + d[2]), duration=0.4)
            tcp = robot.backend.end_effector_pose()
            status = "" if ok else "  (rejected — envelope/IK limit)"
            print(f"  TCP ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}){status}")
            continue
        if cmd == "step" and len(parts) >= 2:
            try:
                jog_step = max(0.001, min(0.10, float(parts[1])))
                print(f"  jog step = {jog_step*1000:.0f} mm")
            except ValueError:
                print("  ? step wants a number in metres, e.g.  step 0.01")
            continue
        if cmd == "teach" and len(parts) >= 2:
            point = engine.teach_point(parts[1], robot)
            engine.save_points(_POINTS_FILE)
            print(f"  taught {point.name!r} at TCP "
                  f"({point.tcp[0]:+.3f}, {point.tcp[1]:+.3f}, {point.tcp[2]:+.3f})")
            continue
        if cmd == "points":
            if not engine.point_names():
                print("  (no taught points yet — jog into position, then `teach NAME`)")
            for n in engine.point_names():
                p = engine.get_point(n)
                print(f"  {n:20s} TCP ({p.tcp[0]:+.3f}, {p.tcp[1]:+.3f}, {p.tcp[2]:+.3f})")
            continue
        if cmd == "objects":
            objs = robot.backend.object_positions()
            if not objs:
                print("  (no pickable objects in this scene, or backend has no perception)")
            else:
                # Two views of every object: absolute base-frame coordinates
                # (what you type into cart/goto) and the delta FROM the
                # gripper (how far you would have to move to reach it).
                tcp = robot.backend.end_effector_pose()
                print("  name                 base frame (x, y, z)        from gripper")
                for name in sorted(objs):
                    x, y, z = objs[name]
                    dx, dy, dz = x - tcp[0], y - tcp[1], z - tcp[2]
                    rel = _describe_offset(dx, dy, dz)
                    print(f"  {name:20s} ({x:+.3f}, {y:+.3f}, {z:+.3f})   {rel}")
            continue
        if cmd in ("pick", "place") and len(parts) >= 2:
            # Resolve the target: a taught point first, then a live scene
            # object by name (in sim, perception is free: the physics state
            # knows every object pose).
            target = None
            try:
                pt = engine.get_point(parts[1])
                target = pt.tcp
            except SkillNotFoundError:
                objs = robot.backend.object_positions()
                if parts[1] in objs:
                    target = tuple(objs[parts[1]])
            if target is None:
                print(f"  no taught point or scene object named {parts[1]!r}")
                print(f"  taught: {engine.point_names()}")
                print(f"  objects: {sorted(robot.backend.object_positions())}")
                continue
            # The Feetech arm cannot put the TCP lower than ~0.115 m at
            # working reach (IK + joint limits), so clamp the grasp height.
            grasp_z = max(float(target[2]), _GRASP_MIN_Z)
            skill_cls = Pick if cmd == "pick" else Place
            skill = skill_cls(x=float(target[0]), y=float(target[1]), z=grasp_z)
            print(f"  running {skill.describe()} ...")
            result = engine.run(skill, robot)
            print(f"  → {result.status.value}"
                  + (f": {result.detail}" if result.detail else ""))
            continue
        if cmd == "open":
            robot.open_gripper()
            continue
        if cmd == "close":
            robot.close_gripper()
            continue

        if cmd == "done":
            if save_name and saved_to is None:
                saved_to = session.save(f"skills/{save_name}.json")
            break
        elif cmd == "help":
            print(_INTERACTIVE_HELP)
        elif cmd == "keys":
            print(_COORDS_HELP)
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
        elif cmd == "where":
            tcp = robot.backend.end_effector_pose()
            joints = robot.backend.joint_positions
            print(f"  TCP    : x={tcp[0]:+.3f}  y={tcp[1]:+.3f}  z={tcp[2]:+.3f}  metres (base frame)")
            print("  Joints : [" + ", ".join(f"{j:+.3f}" for j in joints) + "] rad")
            objs = robot.backend.object_positions()
            if objs:
                print("  Nearby :")
                for name in sorted(objs):
                    x, y, z = objs[name]
                    print(f"    {name:18s} {_describe_offset(x-tcp[0], y-tcp[1], z-tcp[2])}")
        elif cmd == "goto" and len(parts) >= 4:
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            ok = robot.move_to(x, y, z, duration=1.5)
            tcp = robot.backend.end_effector_pose()
            if ok:
                print(f"  arrived at ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f})  — not recorded")
            else:
                print("  (unreachable or outside safety envelope)")
        elif cmd == "save" and len(parts) >= 2:
            saved_to = session.save(f"skills/{parts[1]}.json")
            print(f"  saved → {saved_to}")
        else:
            print("  ? type 'help' for commands")
    return saved_to


def cmd_teach(args: argparse.Namespace) -> int:
    # The default teaching scene includes two named cubes so the `objects`
    # and `pick cube_orange` workflow works out of the box.
    from loophole_arm.control.scene import default_pickplace_scene
    robot, model, data, home = make_sim_robot(arm=args.arm,
                                              scene=default_pickplace_scene())
    session = TeachSession(robot, name=args.name or "untitled", arm=args.arm)

    viewer_ctx = _maybe_viewer(model, data, robot)
    print(f"\nTeaching '{session.trajectory.name}' on the {args.arm} arm.")

    with viewer_ctx:
        robot.home(home)
        saved_to = _interactive_loop(robot, home, session, save_name=args.name)

    if saved_to:
        print(f"\nSaved skill to {saved_to}")
        print(f"Replay it with:  loophole-arm-teach play {saved_to}")
    return 0


def cmd_connect(args: argparse.Namespace) -> int:
    """Teach a skill against a running ``loophole-armd`` server.

    The sim runs in another terminal; this command opens an interactive
    teach prompt that drives the named robot over the wire. Same prompt,
    same commands as ``loophole-arm-teach teach`` — only the backend changes.
    """
    from loophole_arm.control.controller import RobotController
    from loophole_arm.control.kinematics import TCPSolver
    from loophole_arm.server.remote_backend import RemoteBackend

    print(f"Connecting to {args.host}:{args.port} as {args.robot!r}...")
    backend = RemoteBackend(args.robot, host=args.host, port=args.port)
    backend.connect()
    print(f"Connected. n_arm_joints = {backend.n_arm_joints}")

    try:
        model = backend.kinematic_model()
    except RuntimeError:
        print("ERROR: server did not send a kinematic model; cannot teach over the wire.")
        return 2
    if not backend.arm_joint_names or not backend.tcp_site:
        print("ERROR: server did not report arm joint names / TCP site.")
        return 2

    solver = TCPSolver(model, backend.tcp_site, arm_joint_names=backend.arm_joint_names)
    controller = RobotController(backend=backend, solver=solver, control_hz=20.0,
                                 home_pose=tuple(backend.home_pose))
    controller.enable()

    # Home pose reported by the server from the robot's robot.yaml.
    home = list(backend.home_pose) if backend.home_pose else [0.0] * backend.n_arm_joints
    session = TeachSession(controller, name=args.name or "untitled", arm="feetech")

    print(f"\nTeaching '{session.trajectory.name}' over the wire on robot {args.robot!r}.")
    try:
        saved_to = _interactive_loop(controller, home, session, save_name=args.name)
    finally:
        backend.disconnect()

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

    pc = sub.add_parser("connect",
                        help="Teach over the wire against a running loophole-armd")
    pc.add_argument("robot", help="Name of the robot endpoint on the server (e.g. 'arm')")
    pc.add_argument("--host", default="127.0.0.1")
    pc.add_argument("--port", type=int, default=8765)
    pc.add_argument("--name", help="Skill name (saved to skills/NAME.json on 'done')")
    pc.set_defaults(func=cmd_connect)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_level)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
