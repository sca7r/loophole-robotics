# Loophole Arm

[![CI](https://github.com/sca7r/loophole-robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/sca7r/loophole-robotics/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12-blue)](https://www.python.org)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A reward-hacking demonstration on a production-quality UR5e + Robotiq 2F-85.
The arm is asked to lift a cup; an evolution strategy optimises an open-loop
trajectory; the catch is that the optimizer maximises *exactly* the reward you
write, including any loophole it finds.

Part of [Loophole Robotics](../README.md), a product of Helix.

---

## Hardware in simulation

| Component | Source | DoF |
| --- | --- | --- |
| Universal Robots UR5e | [DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | 6 |
| Robotiq 2F-85 parallel gripper | DeepMind MuJoCo Menagerie | 1 (closure command) |
| Free-body cup on a table | This project | — |

The combined model is composed at runtime with `mujoco.MjSpec`, no hand-edited
XML, so the vendored Menagerie models stay drop-in replaceable.

---

## Install

```bash
git clone https://github.com/helix/loophole-robotics.git
cd loophole-robotics/loophole-arm
make fetch-assets   # vendors UR5e + 2F-85 under assets/menagerie/
make dev            # installs the package + dev tools and registers hooks
```

Python 3.10+ required.

---

## Quickstart

```bash
# Inspect the composed scene
make scene

# Train with the naive (hackable) reward, watch the arm fling the cup
make train-naive

# Train with the shaped reward, the cup actually stays near the gripper
make train-shaped

# Render a result to MP4
make render-naive
```

Every run lands under `runs/<timestamp>_<reward>/`:

```
runs/20250101-120000_naive_peak_height/
├── best_params.npy
├── history.json
├── config.json
└── summary.json
```

---

## CLI

```bash
loophole-arm scene --inspect
loophole-arm train  --config configs/naive.yaml
loophole-arm train  --reward shaped_lift --generations 80 --seed 7
loophole-arm render --params runs/.../best_params.npy --out movie.mp4
```

---

## Configuration

Configs are typed YAML loaded into Python dataclasses. Override on the CLI
without editing files:

```yaml
# configs/naive.yaml
reward: naive_peak_height
env:
  n_waypoints: 6
  sim_seconds: 3.0
optimizer:
  population: 32
  elite: 8
  sigma: 0.6
  generations: 60
  seed: 1
```

---

## Architecture

```
src/loophole_arm/
├── scene.py        # MjSpec composition: UR5e + 2F-85 + table + cup
├── env.py          # CupLiftEnv: deterministic rollout, structured result
├── rewards.py      # Reward registry, each fn: RolloutResult -> float
├── optimizer.py    # EvolutionStrategy: gradient-free, NumPy only
├── renderer.py     # Headless MP4 rendering (OSMesa)
├── config.py       # Typed configs + YAML loader
├── cli.py          # Sub-commands: train | render | scene
└── logging_setup.py
```

The four core abstractions are deliberately decoupled, swap the optimizer
(e.g. for CMA-ES or PPO) without touching the environment, or swap the model
(e.g. for a Franka) without touching the optimizer.

---

## Reward functions

| Name | Definition | Behaviour |
| --- | --- | --- |
| `naive_peak_height` | `peak cup z` | Flings the cup upward |
| `shaped_lift` | `final z × exp(-10·dist)` | Holds the cup near the gripper |
| `strict_grasp` | `shaped_lift + contact bonus − motion penalty` | Stays in contact, less wobble |

Add a new reward by writing `def my_reward(r: RolloutResult) -> float:` in
`rewards.py` and registering it in `REGISTRY`. It's available everywhere.

---

## Testing

```bash
make test       # pytest
make lint       # ruff
make typecheck  # mypy --strict
```

---

## Sim → real path

1. **Tune in sim** — thousands of rollouts cost nothing and break nothing.
2. **Swap the model** — `SceneConfig` and `scene.build_spec` are the only
   touchpoints. Drop a different Menagerie arm in and adjust ranges.
3. **Close the loop** — open-loop trajectories will not survive contact with
   real hardware. Add proprioceptive or visual feedback at minimum.
4. **Domain randomization** — vary friction, mass, cup position in sim.
5. **Hardware** — a real UR5e or a lower-cost open-source arm
   (e.g. [LeRobot SO-101](https://github.com/huggingface/lerobot)) can replay
   the optimised trajectory after appropriate safety review.

> Real arms move fast. Test at reduced torque/speed with an e-stop in reach
> before running any optimizer-found trajectory.

---

## License

MIT — see [LICENSE](LICENSE). Vendored robot models from DeepMind MuJoCo
Menagerie are redistributed under their original Apache 2.0 licenses.
