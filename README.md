# Loophole Robotics

[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A robotics product by [Helix](https://github.com/helix). Builds on top of
the open-source stack — [LeRobot](https://github.com/huggingface/lerobot),
[MuJoCo](https://github.com/google-deepmind/mujoco),
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) —
rather than reinventing it.

The flagship application is **Loophole Arm**: a LeRobot-compatible 6-DOF
Feetech-servo manipulator with a reward-hacking sim suite.

---

## Applications

| App | Description | Status |
| --- | --- | --- |
| [`loophole-arm`](./loophole-arm) | LeRobot-compatible 6-DOF Feetech arm + MuJoCo reward-hacking sim | 🟢 Active |

---

## Docs

| | |
| --- | --- |
| [HARDWARE_COSTS.md](./docs/HARDWARE_COSTS.md) | Honest hardware costs per capability tier ($1.5 k–$30 k+) |
| [INDUSTRIAL_DEPLOYMENT.md](./docs/INDUSTRIAL_DEPLOYMENT.md) | Production deployment guide |
| [AI_AGENTS.md](./docs/AI_AGENTS.md) | Imitation learning roadmap (BC → DAgger → Diffusion → VLA) |
| [WORKFLOW.md](./docs/WORKFLOW.md) | Day-to-day developer workflow |

---

## Hierarchy

```
Helix
└── Loophole Robotics
    └── Loophole Arm    ← LeRobot plugin + MuJoCo sim
```

---

## License

MIT — see [LICENSE](LICENSE).
