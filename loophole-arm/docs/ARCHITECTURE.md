# Architecture & technology decisions

This document records *why* the stack is built the way it is — specifically the
language split (Python / C++ / C) and where ROS 2 fits. These are deliberate,
staged choices, not defaults.

## The layered model

```
┌──────────────────────────────────────────────────────────────────────┐
│  PYTHON — "the brain"            best-effort, ~10–50 Hz               │
│  Task logic, teach & replay, perception, commands.py, the safety      │
│  POLICY (limits, state machine). Where the team spends ~95% of time.  │
├──────────────────────────────────────────────────────────────────────┤
│  ROS 2 — "the nervous system"   the glue, added when needed           │
│  ros2_control, MoveIt 2, multi-node coordination, RViz, tooling.      │
│  Earns its complexity once there is more than one component to        │
│  coordinate (arm + AMR, multi-station cell, customer ROS 2 facility). │
├──────────────────────────────────────────────────────────────────────┤
│  C++ — "the spine"              hard real-time, 500–1000 Hz           │
│  Deterministic joint-servo loop, the ros2_control hardware interface. │
│  Deadlines Python cannot guarantee live here.                         │
├──────────────────────────────────────────────────────────────────────┤
│  C / firmware — "the reflexes"  on the servo / MCU                    │
│  Torque & velocity clamps that survive a host or kernel crash. The    │
│  final safety authority. Software safety is defense-in-depth above    │
│  this, never a replacement for it.                                    │
└──────────────────────────────────────────────────────────────────────┘
```

## Why this split

**Python for logic, not loops.** Iteration speed and readability matter most
where behaviour is defined and changed often. Task code, the teach/replay
product, and the *safety policy* (what is allowed) live here. Python cannot meet
hard real-time deadlines, so it never runs the low-level servo loop.

**C++ for determinism.** A 500–1000 Hz servo loop with bounded jitter needs a
compiled language and (on hardware) a `PREEMPT_RT` kernel. This is the
`ros2_control` hardware-interface layer. We do not write it until there is real
hardware to close the loop on — until then, LeRobot's Feetech bus driver covers
low-level I/O.

**C / firmware for the last line of safety.** The robotics literature is blunt
about this: *functional* real-time (a healthy Linux process) is not *hard*
real-time. If the host or kernel dies, only a hardware watchdog on the servo/MCU
can clamp torque and velocity. Our Python `SafetyBackend` catches logic errors
early and makes unsafe commands impossible to express through the normal path —
but on hardware it must be backed by a firmware clamp. It is one layer of
defense-in-depth, not the guarantee.

## Why ROS 2 — and why not yet

ROS 2 is the right long-term middleware: it is the world-wide standard, and
`ros2_control` / MoveIt 2 / RViz are exactly the tools this domain needs. The
decision is *timing*, not *whether*.

It is deliberately deferred because:

1. **It is heavy.** DDS, colcon, launch files, and real-time tuning add
   significant surface. Introducing it before the control and safety layers
   work would bury the robotics work under middleware plumbing.
2. **Its value is coordination.** ROS 2 shines when multiple nodes/components
   must talk. Today there is a single control loop and one arm — nothing to
   coordinate yet.
3. **The first product does not need it.** Teach-and-repeat is a tight Python
   loop: record joint states, replay them.

### The bridge: we are already ROS 2-shaped

The current design intentionally mirrors ROS 2 patterns so adoption is a port,
not a rewrite:

| Loophole today                         | ROS 2 equivalent later                     |
| -------------------------------------- | ------------------------------------------ |
| `RobotInterface` (sim/hardware swap)   | `ros2_control` `SystemInterface` plugin    |
| `SafetyBackend` (validate-then-forward)| `SafetyValidator` hardware-interface shim  |
| `TCPSolver` (mink, MuJoCo-native IK)   | MoveIt 2 / Cartesian controller            |
| `commands.py` (task logic)             | a ROS 2 task/action node                   |

When a customer facility standardises on ROS 2, the `SafetyBackend` logic moves
into a hardware-interface plugin almost verbatim, and the Python task layer
becomes a node — because the seams already exist.

## When each layer gets built

| Trigger                                   | What we add                          |
| ----------------------------------------- | ------------------------------------ |
| Now (sim validation)                      | Python control, IK, **safety policy**|
| Physical arm on the bench                 | C++/LeRobot real-time I/O, firmware clamp |
| >1 component, or customer runs ROS 2      | ROS 2 middleware + `ros2_control` plugin |
| Collision-aware planning around obstacles | MoveIt 2                             |

The guiding rule: **add a layer when its specific value is needed, not
speculatively.** Each one carries real complexity cost and must earn it.
