# Changelog

All notable changes are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.3.0] — control stack: sim-to-real, mink IK, safety, teach-and-repeat

### Added
- **`loophole-arm teach` — the first product: teach-and-repeat by waypoints.**
  Teach a skill in simulation (no hardware needed), save it as a portable JSON
  trajectory, and replay it on demand — in sim now, on the physical arm later,
  unchanged. New module `loophole_arm.teach`:
  - `Trajectory` / `Waypoint` — versioned, human-editable JSON skill format
    (joint / cartesian / gripper / dwell steps).
  - `TeachSession` — builds a trajectory by moving the arm and capturing poses;
    `capture()` is the universal teach primitive (snapshot current joint state).
  - `TrajectoryPlayer` — replays through the same controller, inheriting smooth
    interpolation and full safety enforcement; identical in sim and hardware.
  - `loophole-arm-teach` CLI: `teach` (interactive), `play` (replay, `--render`
    to MP4), `show` (inspect), `demo` (scripted teach + replay for pitches).
  - `tests/test_teach.py` covering the data model and the full
    teach→save→load→repeat loop.

### Added
- **Safety supervisor** (`control/safety.py`, `control/limits.py`). A
  `SafetyBackend` decorator wraps any `RobotInterface` and is itself one, so it
  is transparent to the controller — the same decorator shape as a
  `ros2_control` safety validator, for a clean future port. Enforces, every
  tick: an `IDLE → OPERATIONAL → ESTOP/FAULT` state machine, hard joint limits,
  per-tick velocity (rate) limits, a Cartesian workspace envelope, and a
  latching e-stop. On by default for sim and hardware; opt-out (`safety=False`)
  only for the reward-hacking experiments. Controller gains `enable()`,
  `estop()`, `reset_safety()` pass-throughs.
- **`tests/test_safety.py`** — state-machine transitions, e-stop halt, recovery,
  workspace rejection, gross-command fault, and velocity capping.
- **`docs/ARCHITECTURE.md`** — the layered language split (Python / ROS 2 / C++
  / C-firmware) and the staged rationale for when ROS 2 enters.

### Added (sim-to-real, earlier this cycle)
- **Formal sim-to-real abstraction** (`control/interface.py`). A single
  `RobotInterface` ABC implemented by both `SimBackend` (validation) and
  `HardwareBackend` (deployment); deploying a validated behaviour is a one-line
  backend swap with no change to task code. `commands.py --hardware` shows it.
- **`HardwareBackend`** wrapping the LeRobot Feetech bus; uses the shared MuJoCo
  model for forward kinematics only. Lazy hardware imports.
- **`mink`-based IK** replacing the hand-rolled solver; sub-cm accuracy.
- **TCP site** as the single IK/FK control frame, shared across sim and hardware.
- **`tests/test_sim_to_real.py`** — interface conformance and the FK-consistency
  invariant (sim vs. hardware TCP agree to <0.5 mm).

### Changed
- `RobotController.move_to` seeds IK from `backend.joint_positions` (through the
  interface) instead of MuJoCo `data.qpos`; added a pre-flight workspace check.
- Backends split into focused modules: `interface.py`, `sim_backend.py`,
  `hardware_backend.py`, `safety.py`, `limits.py`.

## [0.2.0] — LeRobot integration

### Added
- **LeRobot-compatible `LoopholeArm` robot class** wrapping `FeetechMotorsBus`.
  Auto-discovered by the `lerobot` CLI via the `lerobot.robots` entry point —
  `lerobot --robot.type=loophole_arm` "just works" once the package is installed.
- **`LoopholeArmConfig`** registered under the canonical `loophole_arm` name.
  Safe defaults: `disable_torque_on_disconnect=True`, `max_relative_target=10.0`.
- **Optional `[hardware]` install extra** that pulls `lerobot[feetech]`.
  Sim layer remains usable without it.
- **Production Docker image** with multi-stage build, non-root user,
  dialout group, healthcheck, tini PID 1, multi-arch (amd64/arm64).
- **Release CI workflow** — PyPI publishing with trusted-publisher OIDC,
  attestations, version-tag verification.
- **Docker CI workflow** — GHCR publishing with SBOM and provenance
  attestation, multi-arch builds.
- **`tests/test_lerobot_integration.py`** — verifies registration, motor
  layout, safety defaults, schema consistency, and discoverability.
- **`docs/HARDWARE_COSTS.md`** — honest cost tiers ($1.5 k–$30 k+) with
  the explicit recommendation that Tier 1 + Tier 2 cover 95% of industrial
  use cases.
- **`docs/INDUSTRIAL_DEPLOYMENT.md`** — 15-minute new-cell bringup
  checklist, safety requirements, observability stack, rollback procedure.

### Changed
- **Reorganised package layout**: sim code moved to `loophole_arm.sim.*`;
  hardware code in `loophole_arm.robot` and `loophole_arm.robot_config`.
- **Lazy LeRobot imports** in `__init__.py` so the sim layer doesn't pull
  LeRobot's transitive deps.
- **CLI scope reduced** to sim-only operations (`loophole-arm-sim`).
  Real-arm operations delegate to `lerobot` CLI.
- **Makefile** rewritten with `help` target and clear hardware-vs-sim
  separation; `hw-calibrate` and `hw-teleop` delegate to `lerobot` CLI.

### Removed
- **C++ `FeetechHardwareInterface`** (`ros2_control` plugin) — replaced by
  LeRobot's `FeetechMotorsBus`, which already does this and is maintained.
- **Custom MuJoCo↔ROS 2 bridge node** — LeRobot already integrates the sim.
- **Custom trajectory runner & policy ROS 2 nodes** — `lerobot-record` and
  `lerobot-train` cover these.
- **Custom YAML run-config + `loophole-arm.config` module** — replaced by
  CLI arguments and LeRobot's `draccus`-based config system.
- **Custom URDF converter (`convert_feetech_urdf.py`)** — the converted
  URDF is now a checked-in artifact (`arm_mujoco.urdf`).

The deletions reflect a deliberate refactor: **build on top of the
open-source stack; don't parallel-implement it.**

## [0.1.0] — initial release

### Added
- Production scene composition: UR5e + Robotiq 2F-85 + table + free-body cup
  via `mujoco.MjSpec`.
- `CupLiftEnv` open-loop rollout environment with structured `RolloutResult`.
- `EvolutionStrategy` optimizer (gradient-free, NumPy only).
- Reward registry: `naive_peak_height`, `shaped_lift`, `strict_grasp`.
- Feetech 6-DOF arm support, default arm selection.
- Headless MP4 renderer.
- Tests, ruff, mypy, GitHub Actions CI matrix.
