# Development workflow

How to actually use this codebase day-to-day. This is the workflow you
described: **run a script, watch the robot in the simulator, read the logs,
fix something, run again.**

---

## The mental model

```
┌─────────────────────────────────────────────────────────────────┐
│                          Your laptop                            │
│                                                                 │
│  Terminal 1: launch sim          Terminal 2: trigger actions    │
│  ──────────────────────          ─────────────────────────      │
│  $ ros2 launch ...sim            $ ros2 service call /train     │
│                                  $ ros2 service call /lift_cup  │
│  [logs streaming]                                               │
│  [RViz window]                   Terminal 3: inspect data       │
│  [MuJoCo viewer]                 ─────────────────────────      │
│                                  $ ros2 topic echo /joint_states│
│                                  $ ros2 bag record -a           │
└─────────────────────────────────────────────────────────────────┘
```

Two layers run side-by-side:

1. **MuJoCo bridge** holds the simulator and publishes ROS topics
2. **Policy node** trains / runs the optimizer and publishes trajectories

Everything else (RViz, MoveIt, rosbag) plugs in as standard ROS 2 tools.

---

## The 90-second dev loop

```bash
# Terminal 1 — bring up the simulator
ros2 launch loophole_arm_ros2 sim.launch.py reward:=shaped_lift

# Terminal 2 — train and execute
ros2 service call /train    std_srvs/srv/Trigger
ros2 service call /lift_cup std_srvs/srv/Trigger

# Terminal 3 — watch what happened
ros2 topic echo /joint_states --once
ros2 bag record -o run_$(date +%s) /joint_states /joint_trajectory_controller/joint_trajectory
```

To iterate:

1. Edit `loophole_arm/rewards.py`, `loophole_arm/scene.py`, or your config
2. `Ctrl+C` Terminal 1, restart it
3. Re-run `/train` and `/lift_cup`
4. Compare to previous bag with `rqt_bag` or `plotjuggler`

---

## Where to look when something is wrong

| Symptom | First place to look | Likely cause |
|---|---|---|
| Arm doesn't move | `ros2 topic list` shows `/joint_trajectory_controller/joint_trajectory`? | Bridge isn't subscribing |
| Arm twitches violently | MuJoCo bridge logs `qacc` warning | Setpoint jumps too large — lower `optimizer.sigma` |
| `/train` hangs | Bridge consuming all CPU | Reduce `n_waypoints` or `generations` |
| Cup teleports off table | `ros2 topic echo /joint_states` shows extreme positions | Optimizer hacked the reward — paste the trajectory file in the issue |
| Tests pass, real arm fails | `ros2 bag info` on the latest bag | Sim-to-real gap — domain randomize |

---

## Logging — what each layer writes

The codebase uses three logging layers. They all stream to your terminal but
have different scopes:

```python
# 1. Python application logging (loophole_arm)
import logging
logger = logging.getLogger(__name__)
logger.info("optimizer complete: best=%.4f", result.best_reward)

# 2. ROS 2 node logging (loophole_arm_ros2)
self.get_logger().info(f"published {len(msg.points)} waypoints")

# 3. rclcpp logging (feetech_hw_interface)
RCLCPP_INFO(rclcpp::get_logger("FeetechHW"), "Connected on %s", port.c_str());
```

To filter the noise in a busy dev session:

```bash
# Show only your nodes' output, hide ros2_control chatter
ros2 launch loophole_arm_ros2 sim.launch.py log_level:=info 2>&1 \
    | grep -E "loophole|mujoco_bridge"

# Or use the per-node log file
~/.ros/log/<timestamp>/mujoco_bridge.log
```

---

## Saving and replaying runs

Every training run writes structured artifacts under `runs/<stamp>_<reward>/`:

```
runs/20251002-143000_shaped_lift/
├── config.json          # exact hyperparameters used
├── best_params.npy      # the learned trajectory (loadable later)
├── history.json         # best reward per generation (for plotting)
└── summary.json         # final cup z, peak z, tcp distance, contacts
```

To replay a saved trajectory on a different day:

```bash
ros2 run loophole_arm_ros2 trajectory_runner.py --ros-args \
    -p params_path:=runs/20251002-143000_shaped_lift/best_params.npy
```

To diff two runs:

```bash
diff runs/run_a/summary.json runs/run_b/summary.json
```

---

## Sim → real cutover

When you flip from sim to hardware, **only one launch file changes**.

```bash
# Sim
ros2 launch loophole_arm_ros2 sim.launch.py

# Real
ros2 launch loophole_arm_ros2 real.launch.py serial_port:=/dev/ttyUSB0
```

Your trained params (`best_params.npy`) are the same artifact in both worlds.
The trajectory_runner publishes the same `JointTrajectory` message. The only
thing that changes is what's downstream of the controller manager:
`MujocoBridge` in sim, `FeetechHardwareInterface` on hardware.

This is why the architectural decision matters — *the optimizer never knows
which world it's in.*
