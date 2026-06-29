# Loophole Robotics

[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Version](https://img.shields.io/badge/version-0.4.2-blue)](./loophole-arm/CHANGELOG.md)

A robotics product by [Helix](https://github.com/Helix-AI-Robotics). Builds on the
open-source stack, [LeRobot](https://github.com/huggingface/lerobot),
[MuJoCo](https://github.com/google-deepmind/mujoco),
[mink](https://github.com/kevinzakka/mink) (IK).

The flagship application is **Loophole Arm**: a 6-DOF Feetech-servo manipulator
with a layered control stack, a software safety layer, and a sim-to-real design
that lets skills taught in simulation run on hardware unchanged.

---

## Products & status

| Product | Description | Status |
| --- | --- | --- |
| **`loophole-arm teach`** | Teach-and-repeat by waypoints, teach in sim, replay on hardware later | 🟢 Built |
| **`loophole-arm vision`** | Camera-guided manipulation | ⚪ Planned |
| reward-hacking sim suite | The original research demo (evolution strategy + reward hacking) | 🟢 Active |

---

## Architecture at a glance

```
commands.py / teach skill        ← what the robot does (the only thing you edit)
        │
RobotController (3-layer API)    ← joint / task-space (mink IK) / skills
        │
SafetyBackend                    ← limits, velocity caps, workspace, e-stop
        │
RobotInterface                   ← the sim-to-real seam
   ┌────┴────┐
SimBackend   HardwareBackend     ← validate in MuJoCo → deploy to Feetech
```

One interface, two backends: a behaviour validated in simulation deploys to the
real arm by swapping the backend, the task code does not change.

---

## Quick start

```bash
cd loophole-arm
pip install -r requirements.txt && pip install -e .
bash scripts/fetch_menagerie.sh        # vendored reference models

# Teach-and-repeat (no hardware needed):
loophole-arm-teach demo                 # teach + replay a pick-and-place
loophole-arm-teach demo --render out.mp4  # render it for a pitch deck

# Live viewer / command file:
python commands.py                      # run the editable task in a live window

# Multi-terminal (one sim, many controllers):
loophole-armd                           # terminal 1: sim server (holds the window)
loophole-arm-teleop arm                 # terminal 2: numpad teleop
loophole-arm-teach connect arm          # terminal 3: teach a skill, live
```

See [`loophole-arm/README.md`](./loophole-arm/README.md) for full details and
[`loophole-arm/src/loophole_arm/teach/README.md`](./loophole-arm/src/loophole_arm/teach/README.md)
for the teach product.

---

## Docs

| | |
| --- | --- |
| [ARCHITECTURE.md](./loophole-arm/docs/ARCHITECTURE.md) | Language split (Python / ROS 2 / C++ / firmware) and when ROS 2 enters |
| [HARDWARE_COSTS.md](./docs/HARDWARE_COSTS.md) | Honest hardware costs per capability tier |
| [INDUSTRIAL_DEPLOYMENT.md](./docs/INDUSTRIAL_DEPLOYMENT.md) | Production deployment guide |
| [AI_AGENTS.md](./docs/AI_AGENTS.md) | Imitation-learning roadmap (BC → DAgger → Diffusion → VLA) |
| [WORKFLOW.md](./docs/WORKFLOW.md) | Day-to-day developer workflow |

---

## Hierarchy

```
Helix
└── Loophole Robotics
    └── Loophole Arm
        ├── teach   (teach-and-repeat)   ← built
        └── vision  (camera-guided)      ← planned
```

---

## License

MIT — see [LICENSE](LICENSE). Vendored MuJoCo Menagerie models retain their
original Apache-2.0 licenses.
