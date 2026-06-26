# Loophole Arm

Part of [Loophole Robotics](../README.md) · A [Helix](https://github.com/helix) product.

A simulated robot arm that optimizes an open-loop trajectory to lift a cup, 
using a plain NumPy evolution strategy in MuJoCo. The catch: the optimizer
maximizes *exactly* the reward you write, loopholes and all.

---

## Quickstart

```bash
pip install mujoco numpy imageio imageio-ffmpeg
python optimize.py     # compare naive vs shaped reward
python render.py       # render result to MP4
```

To view the scene live:
```bash
python -m mujoco.viewer --mjcf=cup_lift.xml
```

---

## Files

| File | Description |
|------|-------------|
| `cup_lift.xml` | MuJoCo scene, 3-DOF arm, 2-finger gripper, free-body cup |
| `optimize.py` | Evolution strategy + two reward functions to compare |
| `render.py` | Renders the best trajectory to MP4 |

---

## The core idea

| Reward | Behavior | Why |
|--------|----------|-----|
| `reward_naive` — peak cup height only | Arm flings the cup | Nothing penalizes throwing |
| `reward_shaped` — height × proximity | Arm grasps and lifts | Harder to game |

Things to try:
1. Reward only cup height → arm throws the cup
2. Add a proximity term → throwing stops
3. Add a motion penalty → pre-grasp wobble cleans up
4. Randomize cup start position → forces generalisation

---

## Tuning knobs

In `optimize.py`:

```python
N_WAYPOINTS = 4       # trajectory resolution
SIM_SECONDS = 2.5     # rollout length
# evolution strategy
generations = 50
pop = 32
elite = 8
sigma = 0.6
```

---

## Sim → real path

1. **Tune in sim** — thousands of free rollouts, nothing breaks
2. **Swap the MJCF** — drop in your real arm's model (URDF/MJCF), match masses and joint limits
3. **Close the loop** — open-loop won't survive real contact; add position feedback at minimum
4. **Domain randomize** — vary friction, mass, cup position in sim before transferring
5. **Hardware** — [LeRobot SO-101](https://github.com/huggingface/lerobot) is a cheap, open-source starting point with an existing MuJoCo model

> ⚠️ Real arms move fast. Test at reduced torque/speed with an e-stop in reach before running any optimizer-found trajectory.

---

## Hierarchy

```
Helix
└── Loophole Robotics
    └── Loophole Arm    ← you are here
```
