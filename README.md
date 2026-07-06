<p align="center">
  <img src="docs/brand/helix-banner.jpeg" alt="Helix: AI, Robotics, Intelligence" width="640">
</p>

# Loophole Robotics

[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Version](https://img.shields.io/badge/version-0.10.0-blue)](./loophole-arm/CHANGELOG.md)
[![Org](https://img.shields.io/badge/by-Helix%20AI%20Robotics-silver)](https://github.com/Helix-AI-Robotics)

The software platform behind Helix's first product: an affordable industrial
pick-and-place robot arm. Everything here runs in simulation today and is
built so the exact same programs run on the physical arm later, unchanged.

This README assumes you are new to robotics AND new to this codebase. It
explains the vocabulary, the mental model, the folder layout, and how to
extend the system without touching existing code.

---

## Robotics vocabulary in 90 seconds

You only need seven terms to read this project.

| Term | Meaning here |
| --- | --- |
| **DOF** (degree of freedom) | One independently controllable axis. Our arm has 6 arm joints + 1 gripper jaw = 7 DOF. |
| **Joint space** | Describing a pose as six joint angles, in radians. Exact but unintuitive. |
| **TCP** (tool center point) | The point between the gripper fingers. "Where the tool is." |
| **Task space** | Describing a pose as the TCP's (x, y, z) in metres. Intuitive but needs math to reach. |
| **IK** (inverse kinematics) | The math that converts a task-space target into joint angles. We use the `mink` solver. |
| **Teach pendant workflow** | The industry way to program arms: jog the arm into position, save the pose under a name, then write programs using names. Nobody types coordinates. |
| **Sim-to-real** | Developing against a physics simulator (MuJoCo) with the guarantee that validated programs run on hardware without modification. |

---

## Quick start (5 minutes, no hardware)

```bash
cd loophole-arm
pip install -r requirements.txt
pip install -e .
bash scripts/fetch_menagerie.sh        # one-time, downloads reference models

loophole-arm-teach demo                # teach + replay a pick-and-place
```

If that ends with `Playback complete`, everything works.

---

## The mental model: layers with one job each

Every layer talks only to the layer below it, through a stable interface.
That single rule is what makes the system portable and extensible.

```
 YOUR PROGRAM            a list of Skills, a teach session, or commands.py
      |
 Skill Engine            Pick, Place, MoveLinear, Home ... parameterised,
      |                  composable primitives. THE product surface.
 Lifecycle FSM           IDLE -> READY -> EXECUTING -> ERROR -> SHUTDOWN.
      |                  Faults become a predictable safe state, not a crash.
 RobotController         three control layers: joints, task space (IK), homing.
      |                  Also robot.health() diagnostics and connect/stop/shutdown.
 SafetyBackend           joint limits, velocity caps, workspace box, e-stop.
      |                  Wraps any backend; refuses unsafe commands.
 RobotInterface          THE seam. A small abstract contract: read joints,
      |                  send targets, move gripper, step time.
      +-- SimBackend         MuJoCo physics (validation)
      +-- HardwareBackend    Feetech servos via LeRobot (deployment)
      +-- RemoteBackend      TCP client to a loophole-armd server
      +-- MockBackend        instant, physics-free (fast tests)
```

Why this matters: a `Pick` skill has no idea whether it is moving simulated
joints, real servos, or a robot in another process. Swapping those is
configuration, not code. `tests/test_skill_portability.py` proves it: the
same skill list runs against two different backends and lands at the same
TCP position.

---

## Repository map

```
loophole-robotics/
├── README.md                  you are here
├── docs/                      business + engineering docs (costs, deployment,
│                              workflow, brand assets)
└── loophole-arm/              the software platform
    ├── robots/                ONE FOLDER PER ROBOT. Everything about a robot
    │   ├── feetech/           lives here: robot.yaml (joints, home pose,
    │   │                      gripper, motor channels, actuation gains,
    │   │                      hardware port), URDF model, meshes/, README.
    │   └── ur5e/              catalog entry for the vendored reference arm.
    ├── examples/scenes/       YAML workcell definitions: which arms, where
    │                          mounted, tables, objects, per-arm safety limits.
    ├── src/loophole_arm/
    │   ├── robots.py          the catalog: load_robot("feetech") -> RobotSpec
    │   ├── skills/            Skill base class, the 11-skill library,
    │   │                      SkillEngine (registry + runner + taught points)
    │   ├── control/
    │   │   ├── interface.py       RobotInterface, THE seam
    │   │   ├── sim_backend.py     MuJoCo implementation
    │   │   ├── hardware_backend.py Feetech implementation (bench-ready)
    │   │   ├── mock_backend.py    physics-free implementation for tests
    │   │   ├── plugins.py         backend registry: create_backend("mock")
    │   │   ├── safety.py          the safety supervisor
    │   │   ├── limits.py          SafetyLimits envelopes
    │   │   ├── controller.py      RobotController + health() diagnostics
    │   │   ├── lifecycle.py       the 5-state FSM
    │   │   ├── kinematics.py      mink IK solver wrapper
    │   │   ├── motor_mapper.py    software radians <-> servo encoder counts
    │   │   ├── scene.py           composable Scene (tables, objects, axes)
    │   │   ├── workcell.py        compiles Scene + N arms into one model
    │   │   └── factory.py         make_sim_robot / make_hardware_robot glue
    │   ├── server/
    │   │   ├── protocol.py        versioned JSON wire protocol
    │   │   ├── sim_server.py      the shared-simulation host
    │   │   ├── remote_backend.py  client-side RobotInterface over TCP
    │   │   ├── config.py          scene YAML loader (per-arm safety)
    │   │   ├── cli.py             loophole-armd entry point
    │   │   └── teleop.py          numpad teleoperation client
    │   ├── teach/             teach-and-replay product (record, play, edit)
    │   ├── sim/scene.py       arm spec builders (URDF -> MuJoCo spec)
    │   ├── robot.py           LeRobot plugin adapter (hardware bring-up)
    │   └── _logging.py        one logging setup for every module
    ├── tests/                 100 tests; every subsystem covered
    └── commands.py            editable demo program using the Skill Engine
```

---

## The three CLIs

```
loophole-arm-teach     teach-and-repeat (in-process; also `connect` for remote)
loophole-armd          the simulation server (multi-terminal, multi-arm)
loophole-arm-teleop    numpad teleop client for a running server
```

### Drive the arm by hand

```bash
loophole-armd                          # terminal 1: opens the sim window
loophole-arm-teleop arm                # terminal 2: numpad drives the TCP
```

### Teach like the industry does

```bash
loophole-arm-teach teach
#   jog z-              nudge the gripper down (step SIZE changes increment)
#   jog x+              position over the object
#   teach pick_pose     name the pose
#   jog y-              move over the bin
#   teach place_pose
#   pick pick_pose      approach -> descend -> grasp -> lift
#   place place_pose    approach -> descend -> release -> lift
# taught points persist in workspace/points.json
```

### Run a multi-arm cell from a YAML file

```bash
loophole-armd --scene examples/scenes/pickplace_dual.yaml
# terminal 2:  loophole-arm-teleop arm_a
# terminal 3:  loophole-arm-teach connect arm_b
```

---

## How to extend the system (the whole point)

Each row is a real workflow, and none of them require editing existing code.

| I want to... | Do this |
| --- | --- |
| Add a new robot model | Copy `robots/feetech/`, replace the URDF + meshes, edit `robot.yaml` (joints, home, gripper, gains). Load it by folder name. |
| Add a robot to a scene | Add an entry under `arms:` in a scene YAML with `kind: <folder name>` and a mount position. |
| Give one arm tighter safety limits | Add a `safety:` block to that arm's entry in the scene YAML. |
| Add a table or object | Add to `tables:` / `objects:` in the scene YAML. |
| Add a new backend (say, EtherCAT) | Implement `RobotInterface` in one file, then `register_backend("ethercat", loader)`. Nothing else changes. |
| Add a new skill | Subclass `Skill` as a frozen dataclass, implement `run()`, register it. Composes with everything immediately. |
| Check robot health | `robot.health()` returns the full PRD diagnostics dict. Sim reports honest `None` for hardware-only values. |
| Run everything fast in CI | Use `create_backend("mock")`: no physics, instant, fault-injectable. |

---

## Status

| | |
| --- | --- |
| Skill Engine (11 PRD skills) | working, tested |
| Lifecycle FSM + fault recovery | working, fault-injection tested |
| Teach-and-repeat + taught points | working |
| Multi-terminal server + teleop | working |
| Multi-arm scenes from YAML | working, per-arm safety limits |
| Robot catalog (folder per robot) | working |
| Plugin Manager (sim/hardware/mock/remote) | working |
| Diagnostics `robot.health()` | working, honest sentinels in sim |
| Hardware (Feetech) | structural: backend + motor mapper ready, needs bench calibration |
| Vision / AI | out of scope for v1 by design (PRD) |

104 tests passing, lint clean, dead-code scan clean. One honestly tracked gap: sim grasp contact (see CHANGELOG 0.9.0).

---

## More documentation

| | |
| --- | --- |
| [`loophole-arm/CHANGELOG.md`](./loophole-arm/CHANGELOG.md) | every release, honestly described |
| [`loophole-arm/docs/ARCHITECTURE.md`](./loophole-arm/docs/ARCHITECTURE.md) | language split and when ROS 2 / C++ enter |
| [`loophole-arm/examples/scenes/README.md`](./loophole-arm/examples/scenes/README.md) | full scene YAML schema |
| [`loophole-arm/robots/feetech/README.md`](./loophole-arm/robots/feetech/README.md) | the robot folder pattern |
| [`docs/HARDWARE_COSTS.md`](./docs/HARDWARE_COSTS.md) | honest hardware costs per tier |
| [`docs/INDUSTRIAL_DEPLOYMENT.md`](./docs/INDUSTRIAL_DEPLOYMENT.md) | production deployment guide |

---

## License

MIT, see [LICENSE](LICENSE). Vendored MuJoCo Menagerie models keep their
original Apache-2.0 licenses.
