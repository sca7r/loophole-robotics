# Changelog

All notable changes are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.4.4] — Helix brand assets

### Added
- **Helix logo assets** under `docs/brand/`: `helix-banner.jpeg` (banner with
  wordmark, 1536×490), `helix-variants.png` (metallic / white / black /
  app-icon reference sheet), `helix-mark.jpeg` (square icon, 306×290).
- Root README now displays the Helix banner and links the org page
  (https://github.com/Helix-AI-Robotics).

### Changed
- Version bump only; no behaviour changes, no code changes, no test changes.

## [0.4.3] — bug fixes, teach-over-wire, friendlier docs

### Fixed
- **`loophole-arm-teleop` no longer crashes on connect.** After 0.4.1 switched
  the server to the multi-arm builder (which prefixes joint names with the
  endpoint name, e.g. ``arm/Joint_1``), the teleop client still had bare
  joint names hardcoded and died in ``TCPSolver.__init__`` with
  ``KeyError: 'Joint_1'``. The server now reports the prefixed
  ``arm_joint_names``, ``tcp_site``, and ``gripper_actuator`` in its ``hello``
  reply, and teleop uses them. Regression test in
  ``tests/test_teleop_wiring.py``.

### Added
- **`loophole-arm-teach connect <robot>`** — teach a skill against a running
  ``loophole-armd`` server. Same interactive prompt as ``loophole-arm-teach
  teach``; only the backend differs (``RemoteBackend`` instead of in-process
  ``SimBackend``). The shared loop is now in a single ``_interactive_loop``
  helper so local and remote modes can't drift.
- **`examples/scenes/README.md`** — full YAML schema reference with named
  colors, the kind|size cheat-sheet, and a table of the shipped example files.

### Changed
- Root README rewritten as a "do this → see this" tutorial. Quick-start that
  works in 5 minutes, followed by concrete recipes for teleop, teach, and
  multi-arm scenes. No new content — just clearer ordering and copy-paste-
  friendly snippets.

## [0.4.2] — YAML scene configuration, per-arm safety limits

### Added
- **YAML scene configs.** `loophole-armd --scene path/to/cell.yaml` loads
  arms + tables + objects + per-arm safety limits from a human-readable
  config file. The YAML schema mirrors the in-code API one-to-one (no extra
  abstractions). New module `server/config.py` with `load_scene_config()`.
- **Per-arm safety limits.** Each arm can have its own `safety:` block in
  the YAML: workspace envelope, max joint step, joint bounds, joint margin.
  Arms that omit `safety:` fall back to `SafetyLimits.feetech_default()`.
  Wired through `_build_endpoints(handles, per_arm_limits)` into each arm's
  `SafetyBackend` — verified by `test_per_arm_limits_reach_safety_backend`.
- **Three example YAML scenes** under `examples/scenes/`:
  `single_arm.yaml`, `pickplace_dual.yaml`, `handoff_triple.yaml` (3 arms
  laid out for a hand-off chain, each with its own workspace box).
- **`tests/test_config.py`** — 11 tests covering: minimal config, dual-arm
  with per-arm limits, scenes with tables/objects/colors, per-joint velocity
  caps as lists, validation (missing fields, bad dimensions, missing file),
  and an integration test that confirms YAML limits actually reach the
  `SafetyBackend`.

### Changed
- `_build_endpoints(model, data, handles, per_arm_limits=None)` accepts an
  optional per-arm limits dict. Default behaviour (no dict) unchanged.

## [0.4.1] — true multi-arm support

### Added
- **True independent multi-arm scenes.** `loophole-armd --arm arm_a --arm arm_b`
  now builds a scene with two physically distinct arms (14 actuated DoFs total),
  laid out along X with one workbench per arm. Each arm has its own prefixed
  joint/actuator/site namespace (e.g. ``arm_a/Joint_1`` vs ``arm_b/Joint_1``)
  via ``MjSpec.attach``, so a SimBackend bound to one arm cannot affect the
  other. Clients connecting to different endpoints control fully independent
  robots that share the same simulation window.
- `ArmInstance` and `ArmHandle` types in `control/workcell.py`; the new
  `build_multi_arm_model(scene, arms)` returns the compiled model, the spec
  (for XML serialisation), and one handle per arm with already-prefixed names.
- `tests/test_multi_arm.py` — six tests covering: compiled-model integrity
  with prefixed names, two-arm independence, validation of duplicate names
  and slash characters, empty arm-list rejection, and single-arm-via-builder
  parity.

### Changed
- `loophole-armd`'s `_build_endpoints` now takes `ArmHandle`s with prefixed
  names instead of bare robot names. The CLI uses the multi-arm builder for
  every case (the single-arm path goes through it too, simpler than branching).
- `_save_model_for_clients` now writes meshes flat next to the XML, matching
  how `MjSpec.attach` rewrites mesh file references (bare filenames, no
  ``meshes/`` prefix). Remote clients can now load the kinematic model in
  multi-arm scenes.

## [0.4.0] — multi-terminal architecture, motor bridge, keyboard teleop

### Added
- **Server/client architecture (multi-terminal).** `loophole-armd` runs one
  simulation that holds the MuJoCo window; clients connect from other
  terminals to drive named robot endpoints. The wire protocol is the
  `RobotInterface` surface serialised as line-delimited JSON, so the same
  controller / IK / safety / teach code drives sim and remote backends
  unchanged. New modules:
  - `server/protocol.py` — versioned wire protocol (request/response,
    compatibility check)
  - `server/sim_server.py` — TCP listener + physics thread + per-client
    handler threads, optional viewer
  - `server/remote_backend.py` — `RemoteBackend(RobotInterface)`, drop-in
    replacement for `SimBackend`/`HardwareBackend`
  - `server/cli.py` — `loophole-armd` entry point
- **`loophole-arm-teleop` — numpad keyboard teleoperation.** Drive the TCP
  with the numpad (7/8/9 +Z/+Y/open, 4/5/6 -X/home/+X, 1/2/3 close/-Y/-Z),
  with live coordinate display, configurable step sizes, e-stop and reset.
  IK-driven, runs through the standard `RobotController` so safety is on.
- **MotorMapper bridge** (`control/motor_mapper.py`). The structural seam
  between software (radians, URDF kinematic frame) and motor (encoder counts,
  calibration offsets, sign, per-tick velocity caps). Wired into
  `HardwareBackend` reads and writes. Default calibration is a placeholder
  (zero offsets) — real values get measured during bench bring-up.
- **Friendlier teach prompt.** Boxed help banner grouped by intent (moving,
  exploring, gripper, managing). New `keys` command prints the coordinate
  system reference. `where` (print current TCP + joint angles) and `goto`
  (preview-move without recording) shipped in the prior 0.3.0 cycle.
- **`tests/test_server.py`** — 12 new tests: protocol round-trip, version
  compatibility, end-to-end client/server, MotorMapper conversions (identity,
  offset, sign-reversal, velocity clamp).

### Changed
- `Scene` is now the composable stage: tables, objects (free-floating cubes /
  spheres / cylinders), lighting, RGB reference axes, table grids. Robots are
  added separately. `build_workcell_from_scene(scene, arm, mount_pos)` builds
  the model; the old `WorkcellConfig` path still works for single-arm cases.
- Robot coloring: clean industrial off-white body, bright orange accent
  gripper (URDF-import-aware: sets geom `rgba` directly).

### Removed
- `teach/jog.py` (older single-process jog). Superseded by
  `loophole-arm-teleop`, which works against the multi-terminal architecture.

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
