"""Command-line interface — sim-only commands.

For real-hardware operations (teleop, record, train, calibrate), use the
``lerobot`` CLI directly with ``--robot.type=loophole_arm``. This CLI exists
only for the simulation experiments that are unique to this project.

Examples
--------
Inspect the composed sim scene::

    loophole-arm-sim scene

Run a reward-hacking optimisation on the sim arm::

    loophole-arm-sim optimize --reward naive_peak_height --generations 50

Render a saved trajectory to MP4::

    loophole-arm-sim render --params runs/best.npy --out demo.mp4
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

from loophole_arm import __version__
from loophole_arm._logging import configure_logging
from loophole_arm.optimizer import EvolutionStrategy
from loophole_arm.rewards import REGISTRY as REWARDS
from loophole_arm.sim.env import CupLiftEnv
from loophole_arm.sim.scene import SceneConfig, build_model

logger = logging.getLogger("loophole_arm.cli")


@dataclass
class OptimizerSettings:
    """All sim-CLI optimisation hyperparameters in one place."""

    reward: str = "shaped_lift"
    arm: str = "feetech"
    n_waypoints: int = 4          # 4 waypoints = faster iteration; raise to 6 for final runs
    sim_seconds: float = 2.0      # shorter rollout = faster; raise to 3.0 for final runs
    population: int = 24
    elite: int = 6
    sigma: float = 0.5
    sigma_decay: float = 0.97
    init_scale: float = 0.3
    generations: int = 30
    seed: int = 0
    output_dir: Path = field(default_factory=lambda: Path("runs"))


# ── Helpers ─────────────────────────────────────────────────────────────
def _make_env(s: OptimizerSettings) -> CupLiftEnv:
    return CupLiftEnv(
        n_waypoints=s.n_waypoints,
        sim_seconds=s.sim_seconds,
        scene=SceneConfig(arm=s.arm),  # type: ignore[arg-type]
    )


def _make_run_dir(root: Path, reward: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = root / f"{stamp}_{reward}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def _resolve_reward(name: str):
    if name not in REWARDS:
        raise SystemExit(f"unknown reward {name!r}. available: {sorted(REWARDS)}")
    return REWARDS[name]


# ── Commands ────────────────────────────────────────────────────────────
def cmd_scene(args: argparse.Namespace) -> int:
    """Print info about the composed sim scene."""
    model = build_model(SceneConfig(arm=args.arm))
    info = {
        "arm": args.arm,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "actuators": [model.actuator(i).name for i in range(model.nu)],
        "joints": [model.joint(i).name for i in range(model.njnt)],
    }
    print(json.dumps(info, indent=2))
    return 0


def cmd_optimize(args: argparse.Namespace) -> int:
    """Run the evolution strategy and persist artifacts."""
    settings = OptimizerSettings(
        reward=args.reward,
        arm=args.arm,
        generations=args.generations,
        seed=args.seed,
        output_dir=Path(args.output_dir),
    )

    reward_fn = _resolve_reward(settings.reward)
    env = _make_env(settings)

    optimizer = EvolutionStrategy(
        param_dim=env.param_dim,
        population=settings.population,
        elite=settings.elite,
        sigma=settings.sigma,
        sigma_decay=settings.sigma_decay,
        init_scale=settings.init_scale,
        seed=settings.seed,
    )

    run_dir = _make_run_dir(settings.output_dir, settings.reward)
    logger.info(
        "reward=%s | arm=%s | generations=%d | run_dir=%s",
        settings.reward,
        settings.arm,
        settings.generations,
        run_dir,
    )

    report_every = max(1, settings.generations // 10)

    def _on_gen(g: int, best: float, sigma: float) -> None:
        if g % report_every == 0 or g == settings.generations - 1:
            logger.info("gen %3d  best=%+.4f  sigma=%.3f", g, best, sigma)

    # MuJoCo prints QACC instability warnings to stderr while the optimizer
    # explores bad trajectories. This is expected and handled by the NaN guard
    # in env.py — suppress so progress logs stay readable.
    import os as _os

    _null_fd = _os.open(_os.devnull, _os.O_WRONLY)
    _stderr_fd = _os.dup(2)
    _os.dup2(_null_fd, 2)
    try:
        result = optimizer.optimize(
            evaluate=lambda p: env.evaluate(p, reward_fn),
            generations=settings.generations,
            on_generation=_on_gen,
        )
    finally:
        _os.dup2(_stderr_fd, 2)
        _os.close(_null_fd)
        _os.close(_stderr_fd)

    np.save(run_dir / "best_params.npy", result.best_params)
    (run_dir / "history.json").write_text(json.dumps(result.reward_history))
    (run_dir / "config.json").write_text(json.dumps(asdict(settings), default=str, indent=2))

    rollout = env.rollout(result.best_params)
    summary = {
        "best_reward": result.best_reward,
        "final_cup_z": float(rollout.final_cup_pos[2]),
        "peak_cup_z": rollout.peak_cup_z,
        "final_cup_tcp_dist": rollout.final_cup_tcp_dist,
        "arm_path_length": rollout.arm_path_length,
        "contacts_with_cup": rollout.contacts_with_cup,
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    print(json.dumps({"run_dir": str(run_dir), **summary}, indent=2))
    return 0


def cmd_render(args: argparse.Namespace) -> int:
    """Replay a saved trajectory to MP4."""
    from loophole_arm.sim.renderer import render_trajectory

    env = _make_env(OptimizerSettings(arm=args.arm))
    params = np.load(args.params)
    render_trajectory(env, params, args.out, resolution=(args.width, args.height), fps=args.fps)
    print(json.dumps({"video": str(args.out)}, indent=2))
    return 0


# ── Entrypoint ──────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="loophole-arm-sim",
        description="Sim-only commands for Loophole Arm. Use `lerobot` CLI for real hardware.",
    )
    p.add_argument("--version", action="version", version=f"loophole-arm {__version__}")
    p.add_argument("--log-level", default="INFO")
    sub = p.add_subparsers(dest="cmd", required=True)

    p_scene = sub.add_parser("scene", help="Inspect the composed sim scene")
    p_scene.add_argument("--arm", choices=["feetech", "ur5e"], default="feetech")
    p_scene.set_defaults(func=cmd_scene)

    p_opt = sub.add_parser("optimize", help="Run the evolution strategy")
    p_opt.add_argument("--arm", choices=["feetech", "ur5e"], default="feetech")
    p_opt.add_argument("--reward", choices=sorted(REWARDS), default="shaped_lift")
    p_opt.add_argument("--generations", type=int, default=30)
    p_opt.add_argument("--seed", type=int, default=0)
    p_opt.add_argument("--output-dir", default="runs")
    p_opt.set_defaults(func=cmd_optimize)

    p_ren = sub.add_parser("render", help="Render saved params to MP4")
    p_ren.add_argument("--arm", choices=["feetech", "ur5e"], default="feetech")
    p_ren.add_argument("--params", type=Path, required=True)
    p_ren.add_argument("--out", type=Path, default=Path("runs/render.mp4"))
    p_ren.add_argument("--width", type=int, default=640)
    p_ren.add_argument("--height", type=int, default=480)
    p_ren.add_argument("--fps", type=int, default=30)
    p_ren.set_defaults(func=cmd_render)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_level)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
