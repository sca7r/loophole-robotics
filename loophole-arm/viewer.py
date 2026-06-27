# ruff: noqa: E402
"""
Loophole Arm — interactive MuJoCo viewer.

Runs locally on YOUR machine and opens a live 3D window.
The server renders to MP4 because it has no display; this script is for
running on your laptop/desktop where a screen is available.

Usage
-----
# Live scene (drag/rotate the camera with mouse):
    python viewer.py

# Watch a saved trajectory play back in real time:
    python viewer.py --params runs/20260627-xxx_shaped_lift/best_params.npy

# Pick a specific reward to optimise then watch live:
    python viewer.py --reward naive_peak_height --optimize --generations 30

Controls
--------
    Mouse drag   Rotate camera
    Scroll       Zoom
    Right drag   Pan
    Space        Pause / resume
    Backspace    Reset simulation
    Esc          Quit
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

# ── resolve the package -------------------------------------------------
# Support running directly from the repo without `pip install -e .`
_repo_root = Path(__file__).resolve().parent
sys.path.insert(0, str(_repo_root / "src"))

from loophole_arm.sim.env import CupLiftEnv
from loophole_arm.sim.scene import SceneConfig
from loophole_arm.optimizer import EvolutionStrategy
from loophole_arm.rewards import REGISTRY


def _optimize(env: CupLiftEnv, reward: str, generations: int, seed: int) -> np.ndarray:
    """Run the evolution strategy and return the best params."""
    reward_fn = REGISTRY[reward]
    optimizer = EvolutionStrategy(
        param_dim=env.param_dim,
        population=24,
        elite=6,
        sigma=0.5,
        seed=seed,
    )

    print(f"\nOptimizing [{reward}] for {generations} generations...")
    print("(close the viewer window to stop early)\n")

    import os
    # suppress MuJoCo NaN warnings during exploration
    null = os.open(os.devnull, os.O_WRONLY)
    saved = os.dup(2)
    os.dup2(null, 2)
    try:
        result = optimizer.optimize(
            evaluate=lambda p: env.evaluate(p, reward_fn),
            generations=generations,
            on_generation=lambda g, b, s: (
                os.dup2(saved, 2),
                print(f"  gen {g:3d}  best={b:+.4f}  sigma={s:.3f}"),
                os.dup2(null, 2),
            ) if g % max(1, generations // 10) == 0 else None,
        )
    finally:
        os.dup2(saved, 2)
        os.close(null)
        os.close(saved)

    print(f"\nDone. best_reward={result.best_reward:.4f}")
    return result.best_params


def _replay_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    env: CupLiftEnv,
    params: np.ndarray,
) -> None:
    """Play a trajectory once inside the viewer, then idle."""
    setpoints = env.decode(params)
    dt = model.opt.timestep
    steps_per_wp = max(1, int((env.sim_seconds / env.n_waypoints) / dt))

    mujoco.mj_resetData(model, data)
    data.qpos[: len(env.scene.resolved().home_qpos)] = env.scene.resolved().home_qpos
    mujoco.mj_forward(model, data)

    for wp in range(env.n_waypoints):
        data.ctrl[:] = setpoints[wp]
        for _ in range(steps_per_wp):
            mujoco.mj_step(model, data)
            time.sleep(dt)          # slow down to real time


def _idle_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> None:
    """Step physics at real time so the viewer stays live after replay."""
    dt = model.opt.timestep
    while True:
        mujoco.mj_step(model, data)
        time.sleep(dt)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Loophole Arm — interactive MuJoCo viewer (run locally)"
    )
    parser.add_argument("--arm", choices=["feetech", "ur5e"], default="feetech",
                        help="Which arm to simulate (default: feetech)")
    parser.add_argument("--params", type=Path, default=None,
                        help="Path to best_params.npy from a previous run")
    parser.add_argument("--optimize", action="store_true",
                        help="Run the optimizer before opening the viewer")
    parser.add_argument("--reward", choices=sorted(REGISTRY), default="shaped_lift",
                        help="Reward to optimize (used with --optimize)")
    parser.add_argument("--generations", type=int, default=30,
                        help="ES generations (used with --optimize)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--loop", action="store_true",
                        help="Keep replaying the trajectory in a loop")
    args = parser.parse_args()

    env = CupLiftEnv(
        n_waypoints=4,
        sim_seconds=2.0,
        scene=SceneConfig(arm=args.arm),  # type: ignore[arg-type]
    )
    model = env.model
    data = env.data

    params: np.ndarray | None = None

    if args.params:
        params = np.load(args.params)
        print(f"Loaded params from {args.params}")
    elif args.optimize:
        params = _optimize(env, args.reward, args.generations, args.seed)
    else:
        print("Opening viewer in free-roam mode.")
        print("Tip: pass --optimize or --params runs/.../best_params.npy to watch a policy.")

    # Launch the live viewer window.
    # mujoco.viewer.launch_passive keeps Python running while the window is open.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.lookat[:] = [0.15, 0.0, 0.15]

        if params is not None:
            print("\nPlaying trajectory — use mouse to rotate camera.")
            while viewer.is_running():
                _replay_loop(model, data, env, params)
                if not args.loop:
                    print("Replay done. Idling — close the window to quit.")
                    break
                print("Replaying...")

        # Keep viewer alive (physics continues stepping)
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    return 0


if __name__ == "__main__":
    sys.exit(main())
