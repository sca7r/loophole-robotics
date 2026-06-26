"""Command-line interface for Loophole Arm.

Usage::

    loophole-arm train  --reward naive_peak_height
    loophole-arm render --reward shaped_lift   --out runs/shaped.mp4
    loophole-arm scene  --inspect
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import numpy as np

from loophole_arm import __version__
from loophole_arm.config import RunConfig
from loophole_arm.env import CupLiftEnv
from loophole_arm.logging_setup import configure_logging
from loophole_arm.optimizer import EvolutionStrategy
from loophole_arm.renderer import render_trajectory
from loophole_arm.rewards import REGISTRY
from loophole_arm.scene import SceneConfig, build_model

logger = logging.getLogger("loophole_arm.cli")


# ---- helpers ---------------------------------------------------------------
def _load_config(path: Path | None) -> RunConfig:
    return RunConfig.from_yaml(path) if path else RunConfig()


def _resolve_reward(name: str):
    if name not in REGISTRY:
        raise SystemExit(
            f"unknown reward {name!r}. Available: {sorted(REGISTRY)}"
        )
    return REGISTRY[name]


def _make_env(cfg: RunConfig) -> CupLiftEnv:
    return CupLiftEnv(
        n_waypoints=cfg.env.n_waypoints,
        sim_seconds=cfg.env.sim_seconds,
    )


def _new_run_dir(root: Path, reward: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = root / f"{stamp}_{reward}"
    out.mkdir(parents=True, exist_ok=True)
    return out


# ---- commands --------------------------------------------------------------
def cmd_train(args: argparse.Namespace) -> int:
    cfg = _load_config(args.config)
    if args.reward:
        cfg.reward = args.reward
    if args.generations:
        cfg.optimizer.generations = args.generations
    if args.seed is not None:
        cfg.optimizer.seed = args.seed

    reward_fn = _resolve_reward(cfg.reward)
    env = _make_env(cfg)

    optimizer = EvolutionStrategy(
        param_dim=env.param_dim,
        population=cfg.optimizer.population,
        elite=cfg.optimizer.elite,
        sigma=cfg.optimizer.sigma,
        sigma_decay=cfg.optimizer.sigma_decay,
        init_scale=cfg.optimizer.init_scale,
        seed=cfg.optimizer.seed,
    )

    run_dir = _new_run_dir(Path(cfg.output_dir), cfg.reward)
    logger.info("run dir: %s", run_dir)
    logger.info("reward: %s | generations: %d | param_dim: %d",
                cfg.reward, cfg.optimizer.generations, env.param_dim)

    def on_gen(g: int, best: float, sigma: float) -> None:
        if g % max(1, cfg.optimizer.generations // 10) == 0 or g == cfg.optimizer.generations - 1:
            logger.info("gen %3d  best=%+.4f  sigma=%.3f", g, best, sigma)

    result = optimizer.optimize(
        evaluate=lambda p: env.evaluate(p, reward_fn),
        generations=cfg.optimizer.generations,
        on_generation=on_gen,
    )

    # Persist artifacts
    np.save(run_dir / "best_params.npy", result.best_params)
    (run_dir / "history.json").write_text(json.dumps(result.reward_history))
    (run_dir / "config.json").write_text(json.dumps(asdict(cfg), indent=2))

    # Final diagnostic rollout
    final_rollout = env.rollout(result.best_params)
    summary = {
        "best_reward": result.best_reward,
        "final_cup_z": float(final_rollout.final_cup_pos[2]),
        "peak_cup_z": final_rollout.peak_cup_z,
        "final_cup_tcp_dist": final_rollout.final_cup_tcp_dist,
        "arm_path_length": final_rollout.arm_path_length,
        "contacts_with_cup": final_rollout.contacts_with_cup,
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    logger.info("summary: %s", json.dumps(summary))
    print(json.dumps({"run_dir": str(run_dir), **summary}, indent=2))
    return 0


def cmd_render(args: argparse.Namespace) -> int:
    cfg = _load_config(args.config)
    if args.reward:
        cfg.reward = args.reward
    if args.generations:
        cfg.optimizer.generations = args.generations

    reward_fn = _resolve_reward(cfg.reward)
    env = _make_env(cfg)

    if args.params:
        params = np.load(args.params)
    else:
        # Quick optimize then render
        optimizer = EvolutionStrategy(
            param_dim=env.param_dim,
            population=cfg.optimizer.population,
            elite=cfg.optimizer.elite,
            sigma=cfg.optimizer.sigma,
            sigma_decay=cfg.optimizer.sigma_decay,
            init_scale=cfg.optimizer.init_scale,
            seed=cfg.optimizer.seed,
        )
        result = optimizer.optimize(
            evaluate=lambda p: env.evaluate(p, reward_fn),
            generations=cfg.optimizer.generations,
        )
        params = result.best_params

    out = Path(args.out)
    render_trajectory(env, params, out, resolution=(args.width, args.height), fps=args.fps)
    print(json.dumps({"video": str(out)}, indent=2))
    return 0


def cmd_scene(args: argparse.Namespace) -> int:
    model = build_model(SceneConfig())
    info = {
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "actuators": [model.actuator(i).name for i in range(model.nu)],
        "joints": [model.joint(i).name for i in range(model.njnt)],
    }
    print(json.dumps(info, indent=2))
    return 0


# ---- entrypoint ------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="loophole-arm")
    p.add_argument("--version", action="version", version=f"loophole-arm {__version__}")
    p.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING, ERROR")

    sub = p.add_subparsers(dest="cmd", required=True)

    pt = sub.add_parser("train", help="Run an optimization")
    pt.add_argument("--config", type=Path, default=None, help="YAML config file")
    pt.add_argument("--reward", choices=sorted(REGISTRY), help="Override reward")
    pt.add_argument("--generations", type=int, default=None)
    pt.add_argument("--seed", type=int, default=None)
    pt.set_defaults(func=cmd_train)

    pr = sub.add_parser("render", help="Render a trajectory to MP4")
    pr.add_argument("--config", type=Path, default=None)
    pr.add_argument("--reward", choices=sorted(REGISTRY))
    pr.add_argument("--generations", type=int, default=None,
                    help="Generations to optimize before rendering (ignored if --params is given)")
    pr.add_argument("--params", type=Path, default=None, help="Load saved best_params.npy")
    pr.add_argument("--out", type=Path, default=Path("runs/render.mp4"))
    pr.add_argument("--width", type=int, default=640)
    pr.add_argument("--height", type=int, default=480)
    pr.add_argument("--fps", type=int, default=30)
    pr.set_defaults(func=cmd_render)

    ps = sub.add_parser("scene", help="Print info about the composed scene")
    ps.add_argument("--inspect", action="store_true")
    ps.set_defaults(func=cmd_scene)

    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    configure_logging(args.log_level)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
