# Loophole Arm

[![CI](https://github.com/helix/loophole-robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/helix/loophole-robotics/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/loophole-arm)](https://pypi.org/project/loophole-arm/)
[![Python](https://img.shields.io/pypi/pyversions/loophole-arm)](https://pypi.org/project/loophole-arm/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

LeRobot-compatible 6-DOF Feetech-servo manipulator with a MuJoCo-based
reward-hacking demonstration suite.

Part of [Loophole Robotics](../README.md), a product by [Helix](https://github.com/helix).

---

## What it provides

```
┌──────────────────────────────────────────────────────────────┐
│  loophole_arm (this package)                                 │
│  ─────────────────────────────                               │
│  • Robot class    →  registered with LeRobot CLI             │
│  • RobotConfig    →  registered as "loophole_arm"            │
│  • MuJoCo sim     →  scene composition + rollout env         │
│  • Reward funcs   →  reward-hacking demonstrations           │
│  • ES optimizer   →  gradient-free policy search             │
└──────────────────────────────────────────────────────────────┘
       │ extends                              │ uses
       ▼                                      ▼
┌─────────────────┐                ┌────────────────────┐
│ LeRobot (HF)    │                │ MuJoCo + numpy     │
│ • teleop/record │                │ • physics & assets │
│ • train ACT/Pi0 │                │                    │
│ • Feetech bus   │                │                    │
└─────────────────┘                └────────────────────┘
```

We deliberately don't reinvent what LeRobot does. Hardware control, dataset
recording, teleoperation, and state-of-the-art policy training (ACT,
Diffusion Policy, Pi0, SmolVLA, GR00T) all go through `lerobot`'s CLI with
`--robot.type=loophole_arm`. This package adds the simulator and the
research-grade reward-hacking tooling around it.

---

## Install

### Sim only (CPU, no robot)
```bash
pip install loophole-arm
make fetch-assets        # vendor UR5e + Robotiq from Menagerie
```

### Real hardware (LeRobot + Feetech bus)
```bash
pip install "loophole-arm[hardware]"
```

### Development
```bash
git clone https://github.com/helix/loophole-robotics
cd loophole-robotics/loophole-arm
make dev fetch-assets
```

Requires **Python 3.10+** and Linux/macOS. For real hardware on Linux, your
user must be in the `dialout` group.

---

## Quickstart

### Inspect & optimize in sim
```bash
loophole-arm-sim scene                                   # show the composed scene
loophole-arm-sim optimize --reward shaped_lift           # train a trajectory in MuJoCo
loophole-arm-sim render --params runs/.../best_params.npy --out demo.mp4
```

### Drive the real arm (via LeRobot CLI — same robot type works for all commands)
```bash
# One-time servo bus setup
lerobot-setup-motors --robot.type=loophole_arm --robot.port=/dev/ttyUSB0

# Calibration (interactive)
lerobot-calibrate    --robot.type=loophole_arm --robot.port=/dev/ttyUSB0

# Teleop from a leader arm
lerobot-teleoperate  --robot.type=loophole_arm --robot.port=/dev/ttyUSB0 \
                     --teleop.type=so100_leader --teleop.port=/dev/ttyUSB1

# Record demonstrations (uploaded to HF Hub by default)
lerobot-record       --robot.type=loophole_arm --robot.port=/dev/ttyUSB0 \
                     --dataset.repo_id=<user>/my-demos --dataset.num_episodes=50

# Train an ACT policy on those demos
lerobot-train --policy=act --dataset.repo_id=<user>/my-demos
```

### Docker
```bash
make docker-build
docker run --rm --device=/dev/ttyUSB0 loophole-arm:dev \
    lerobot-record --robot.type=loophole_arm --robot.port=/dev/ttyUSB0 ...
```

---

## Architecture

```
src/loophole_arm/
├── robot.py            LeRobot Robot subclass wrapping FeetechMotorsBus
├── robot_config.py     LeRobot RobotConfig, registered as "loophole_arm"
├── sim/
│   ├── scene.py        MuJoCo scene composition (Feetech / UR5e + cup)
│   ├── env.py          Deterministic rollout environment
│   └── renderer.py     Headless MP4 rendering
├── rewards.py          Reward function registry
├── optimizer.py        NumPy evolution strategy (gradient-free)
├── sim_cli.py          `loophole-arm-sim` CLI (sim only)
└── _logging.py         Structured logging setup
```

The package is split deliberately:

- The **sim layer** (`sim/`, `rewards.py`, `optimizer.py`) has no LeRobot
  dependency — it imports cleanly without the `[hardware]` extra. Useful
  on CI and on machines without a Feetech bus.
- The **hardware layer** (`robot.py`, `robot_config.py`) is lazy-imported.
  Only loads `lerobot.motors.feetech` when actually used.

---

## Tests

```bash
make test            # all sim tests
make test-cov        # with coverage report
make typecheck       # mypy --strict
make lint            # ruff
make pre-commit      # everything via pre-commit
```

Hardware-in-the-loop tests are marked `@pytest.mark.hardware` and skip
unless a real arm is on `/dev/ttyUSB0`. They're not run in CI.

---

## Configuration

The Loophole Arm is a standard LeRobot `RobotConfig` subclass:

```python
from loophole_arm import LoopholeArm, LoopholeArmConfig

cfg = LoopholeArmConfig(
    port="/dev/ttyUSB0",
    disable_torque_on_disconnect=True,
    max_relative_target=10.0,         # degrees per tick — velocity-limit safety
    use_degrees=True,
)
arm = LoopholeArm(cfg)
arm.connect()
obs = arm.get_observation()
arm.send_action({"shoulder_pan.pos": 0.0, ...})
arm.disconnect()
```

For YAML config in `lerobot-record`, just point `--robot.type=loophole_arm`
and pass the corresponding `--robot.port`, `--robot.id` etc.

---

## Sim reward hacking — the research tooling

Three rewards demonstrate how the optimizer exploits whatever you write:

| Reward | Behaviour |
|---|---|
| `naive_peak_height` | Optimizer learns to fling the cup (height ↑, holding ↓) |
| `shaped_lift` | Cup must end near the gripper — flinging stops |
| `strict_grasp` | Adds contact-time bonus + motion penalty — cleanest motion |

This isn't a toy — it's a calibration check: before deploying a policy, run
the optimizer against your reward to surface edge cases.

```bash
loophole-arm-sim optimize --reward strict_grasp --generations 100
```

---

## Hardware costs

See [../docs/HARDWARE_COSTS.md](../docs/HARDWARE_COSTS.md) for the honest
breakdown. TL;DR: **$1.5 k–$3 k per deployment cell**, **$4 k–$7 k for a
training workstation**. Anything beyond Tier 2 is research/SOTA chasing and
not required for shipping.

---

## Industrial deployment

See [../docs/INDUSTRIAL_DEPLOYMENT.md](../docs/INDUSTRIAL_DEPLOYMENT.md) for:

- 15-minute new-cell bringup checklist
- Safety requirements (hardware, software, operational)
- Observability stack
- Rollback procedure
- When to add ROS 2 (and when not to)

---

## License

MIT — see [LICENSE](LICENSE). Vendored MuJoCo Menagerie models stay under
their original Apache 2.0 licenses.
