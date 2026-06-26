# Changelog

All notable changes are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.0] — initial release

### Added
- Production scene composition: UR5e + Robotiq 2F-85 + table + free-body cup
  via `mujoco.MjSpec`.
- `CupLiftEnv` open-loop rollout environment with structured `RolloutResult`.
- `EvolutionStrategy` optimizer (gradient-free, NumPy only).
- Reward registry: `naive_peak_height`, `shaped_lift`, `strict_grasp`.
- CLI: `loophole-arm train | render | scene`.
- YAML configs and run artifact persistence under `runs/<timestamp>_<reward>/`.
- Headless MP4 renderer.
- Tests, ruff, mypy, GitHub Actions CI matrix (3.10 / 3.11 / 3.12).
