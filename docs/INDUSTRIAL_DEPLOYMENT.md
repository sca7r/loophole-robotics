# Industrial deployment

A practical guide for shipping Loophole Arm to a production environment.

---

## Mental model

```
        ┌──────────────────┐                  ┌────────────────────┐
        │  Training cell   │ ── HF Hub ────▶  │ Deployment cell(s) │
        │  (Tier 2 GPU)    │   datasets +     │ (Tier 1 CPU)       │
        │                  │   checkpoints    │                    │
        │ • LeRobot train  │                  │ • Docker container │
        │ • MuJoCo sim     │                  │ • Loophole Arm     │
        │ • Loophole Arm   │                  │ • Policy inference │
        │   sim CLI        │                  │ • LeRobot record   │
        └──────────────────┘                  └────────────────────┘
                                                     │
                                              ┌──────▼───────┐
                                              │ Feetech arm  │
                                              │ + camera     │
                                              └──────────────┘
```

**One central training workstation.** **Many edge cells** running CPU-only
inference inside the Docker image.

---

## Bringing up a new cell (15-minute checklist)

1. **Plug it in.** Arm → USB-to-serial → industrial PC. Camera if applicable.

2. **Run the image.**
   ```bash
   docker pull ghcr.io/helix/loophole-robotics/loophole-arm:latest
   docker run -it --rm \
       --device=/dev/ttyUSB0 \
       --device=/dev/video0 \
       --network host \
       -v /home/robot/.cache/huggingface:/home/robot/.cache/huggingface \
       -e HF_TOKEN \
       ghcr.io/helix/loophole-robotics/loophole-arm:latest \
       /bin/bash
   ```

3. **Set servo IDs (one-time per arm).**
   ```bash
   lerobot-setup-motors --robot.type=loophole_arm --robot.port=/dev/ttyUSB0
   ```

4. **Calibrate the arm (one-time per assembly).**
   ```bash
   lerobot-calibrate --robot.type=loophole_arm --robot.port=/dev/ttyUSB0 \
                     --robot.id=cell_$(hostname -s)
   ```
   Calibration is persisted under `~/.cache/huggingface/lerobot/calibration/`
   so subsequent boots skip this step.

5. **Pull the latest policy.**
   ```bash
   huggingface-cli download helix/loophole-arm-cup-lift --local-dir ./policy
   ```

6. **Run inference.**
   ```bash
   lerobot-record \
       --robot.type=loophole_arm \
       --robot.port=/dev/ttyUSB0 \
       --robot.id=cell_$(hostname -s) \
       --policy.path=./policy \
       --dataset.repo_id=helix/cell-${HOSTNAME}-eval-$(date +%Y%m%d) \
       --dataset.num_episodes=20
   ```

That's it. The cell is live and uploading evaluation episodes back to the
hub for further training.

---

## Safety: non-negotiable

Industrial deployment has a different threshold than a research demo.
Implement these *before* the arm sees production data:

### Hardware-level safety

- **E-stop on the power line.** Cuts servo power, not just torque. Hardware
  switch within arm's reach of any person it might interact with.
- **Workspace fence.** Even with sub-3 N·m servos, the arm can pinch.
  Physical barrier > software policy.
- **Inrush limit.** A 5 A fuse on the 12 V line protects against shorted
  servos.

### Software-level safety

The defaults in `LoopholeArmConfig` already enforce these — don't override
them without a written justification:

- `disable_torque_on_disconnect = True` — graceful shutdown drops servos
  limp rather than locking them at the last commanded position.
- `max_relative_target = 10.0` — caps per-tick joint motion (≈ 10° in
  degree mode). Anything beyond this is rejected before reaching the bus.

### Operational safety

- **Run policies in shadow mode first.** Let the policy compute actions for
  a week without commanding them; compare against teleop traces. Watch for
  divergence.
- **Anomaly thresholds.** Track joint current and contact forces. Trip the
  e-stop relay if either exceeds the empirical baseline by 3σ.
- **Versioned policies.** Tag every deployed checkpoint. When something
  breaks, you need to roll back exactly. The Hugging Face hub revision
  system handles this; pin to a SHA, not `main`.

---

## Observability

Loophole Arm uses standard Python logging + LeRobotDataset for trajectories,
so any observability stack that speaks those formats works. Suggested setup:

| Concern | Tool | What it sees |
|---|---|---|
| Container health | Docker healthcheck | `loophole-arm-sim --version` |
| Joint-level telemetry | Prometheus exporter (DIY, ~50 lines) | per-joint position, velocity, current |
| Trajectory replay & inspection | LeRobotDataset on Hugging Face Hub | full state/action/observation episodes |
| Crash diagnostics | `journalctl -u loophole-arm` (or `docker logs`) | structured Python logs |

A minimal Prometheus exporter (CPU-only, no extra deps) is on the roadmap.

---

## Rollback procedure

When a deployed policy misbehaves:

```bash
# 1. Park the arm (idempotent, safe)
lerobot-park --robot.type=loophole_arm --robot.port=/dev/ttyUSB0

# 2. Roll the container image back to the last green tag
docker pull ghcr.io/helix/loophole-robotics/loophole-arm:v0.4.2  # the known-good tag
docker compose up -d                                              # if using compose

# 3. Roll the policy back via the hub revision SHA
huggingface-cli download helix/loophole-arm-cup-lift \
    --revision=$LAST_KNOWN_GOOD_SHA --local-dir ./policy
```

Every Docker tag is signed (see `docker.yml` — `attestations: true`); verify
the image before running on customer hardware:

```bash
docker buildx imagetools inspect \
    ghcr.io/helix/loophole-robotics/loophole-arm:latest \
    --format "{{json .Provenance}}"
```

---

## When to add ROS 2

The current architecture works *without* ROS 2 because LeRobot owns the
control-loop abstraction. Add ROS 2 only when one of these is true:

- The arm is part of a multi-robot system (AMRs + manipulators talking via
  topics).
- The customer's facility already standardised on ROS 2 — your software has
  to coexist with theirs.
- You need MoveIt 2 for collision-aware planning with complex obstacles.

For a stand-alone Feetech-arm pick-and-place cell, ROS 2 is overhead with
no payoff. Don't add it speculatively.

The integration point if/when you do: write a small `loophole_arm_ros2`
node that wraps `LoopholeArm` and re-exposes its observations/actions as
ROS 2 topics. Roughly 150 lines.
