# Loophole Robotics

[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A robotics product by Helix. Loophole Robotics
explores what happens when you give a robot a reward function and get out of
the way, physics simulation, evolution strategies, and eventually real
hardware, all built around one idea: **the optimizer will always find the
loophole.**

---

## Applications

| App | Description | Status |
| --- | --- | --- |
| [`loophole-arm`](./loophole-arm) | Reward-hacking on a UR5e + Robotiq 2F-85 in MuJoCo | 🟢 Active |

---

## Repository layout

```
loophole-robotics/
├── loophole-arm/      # Application: cup-lift, evolution strategy, reward registry
├── shared/            # Common utilities, scenes, configs reused across apps
├── LICENSE
└── README.md
```

Each application is a self-contained Python project with its own dependencies,
tests, CI, and CLI. Code shared between two or more apps moves into `shared/`
and is referenced as a path-installed dependency.

---

## Hierarchy

```
Helix
└── Loophole Robotics
    └── Loophole Arm
```

---

## License

MIT — see [LICENSE](LICENSE).
