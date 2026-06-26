# Loophole Robotics

A product by [Helix].

Loophole Robotics explores what happens when you give an optimizer a reward function
and get out of the way. Physics simulation, evolution strategies, and eventually real
hardware, all built around one idea: the optimizer will always find the loophole.

---

## Applications

| App | Description | Status |
|-----|-------------|--------|
| [`loophole-arm`](./loophole-arm) | Simulated robot arm optimized via evolution strategy in MuJoCo | 🟢 Active |

---

## Repo structure

```
loophole-robotics/
├── loophole-arm/        # Cup-lift sim: reward hacking demo
├── shared/              # Common utilities, models, configs (future)
└── README.md
```

## Roadmap

- [ ] Improve grasp physics in `loophole-arm`
- [ ] Add domain randomization for sim-to-real transfer
- [ ] Integrate a real arm (SO-101 / xArm)
- [ ] Second application TBD

---

## Part of

```
Helix
└── Loophole Robotics        ← you are here
    └── Loophole Arm
```
