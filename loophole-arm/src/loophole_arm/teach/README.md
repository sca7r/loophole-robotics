# loophole-arm teach

Teach-and-repeat for the Loophole Arm. Teach the arm a skill by setting
waypoints **in the simulator** — no hardware required — save it as a portable
file, and replay it on demand. The same skill file runs the physical arm later,
unchanged.

This is the first Loophole Arm product. The second, `loophole-arm vision`
(camera-guided), builds on the same foundation later.

---

## Why this works without hardware

"Teaching" a robot is, at its core, **recording joint states**. The simulator
produces joint states exactly as a physical arm would, so you can teach,
validate, and demo a complete skill in software first — then fund and build the
hardware knowing the skill already works.

When the hardware arrives, the recorded skill replays on it through the same
control interface, with no re-teaching. That is the entire point of the
[sim-to-real architecture](../../docs/ARCHITECTURE.md).

---

## The workflow

```
  TEACH (sim)                SAVE              REPEAT (sim now, hardware later)
  ───────────                ────              ────────────────────────────────
  set waypoints       →   skill.json      →   load + replay through the same
  arm moves & you           (portable,          controller, with full safety
  verify each pose          editable JSON)      enforcement, identical motion
```

---

## Quick start

```bash
# Teach + replay an example pick-and-place (the pitch demo):
loophole-arm-teach demo

# Render that demo to a video for a deck:
loophole-arm-teach demo --render pickplace.mp4

# Inspect a saved skill:
loophole-arm-teach show skills/pick_place_demo.json

# Replay a saved skill (loop it 3 times):
loophole-arm-teach play skills/pick_place_demo.json --loops 3
```

### Teach your own skill interactively

```bash
loophole-arm-teach teach --name my_skill
```

Then, at the `teach>` prompt (the arm moves in the live viewer as you go):

```
cart 0.18 0.08 0.18  above pick
cart 0.18 0.08 0.12  descend
grip close
cart 0.18 0.08 0.18  lift
cart 0.18 -0.08 0.12 place
grip open
home
save my_skill
done
```

That writes `skills/my_skill.json`, replayable any time on sim or hardware.

---

## Teaching commands

| Command | Effect |
| --- | --- |
| `cart X Y Z [label]` | Move TCP to (x, y, z) metres via IK, record it |
| `joints J1..J6 [label]` | Move to absolute joint angles (radians), record |
| `grip open` / `grip close` | Actuate and record the gripper |
| `dwell SECONDS` | Record a pause |
| `home` | Move to the home pose and record it |
| `undo` | Remove the last waypoint |
| `list` | Show recorded waypoints |
| `save NAME` | Save to `skills/NAME.json` |
| `done` | Finish and exit |

---

## The skill file

Plain JSON — human-readable, diff-able, hand-editable. Tweak a coordinate or a
duration in a text editor and replay; no re-teaching needed.

```json
{
  "format_version": "1.0",
  "name": "pick_place_demo",
  "arm": "feetech",
  "control_hz": 20.0,
  "waypoints": [
    { "kind": "cartesian", "position": [0.18, 0.08, 0.12],
      "duration": 1.5, "label": "grasp" },
    { "kind": "gripper", "gripper": 1.0, "label": "close" }
  ]
}
```

---

## Programmatic API

```python
from loophole_arm.control import make_sim_robot
from loophole_arm.teach import TeachSession, TrajectoryPlayer, Trajectory

robot, model, data, home = make_sim_robot(arm="feetech")

# Teach
s = TeachSession(robot, name="pick_place", arm="feetech")
s.teach_cartesian(0.18, 0.08, 0.18, label="above")
s.teach_cartesian(0.18, 0.08, 0.12, label="grasp")
s.teach_gripper(1.0)
s.save("skills/pick_place.json")

# Repeat (later, or on hardware via make_hardware_robot)
traj = Trajectory.load("skills/pick_place.json")
TrajectoryPlayer(robot).play(traj)
```

---

## What's enforced during replay

Replay goes through the standard controller, so it inherits the full
[safety layer](../../docs/ARCHITECTURE.md): joint limits, per-tick velocity
limits, workspace bounds, and the e-stop state machine. A taught skill cannot
replay an unsafe or unreachable motion — if a waypoint is rejected, playback
stops cleanly.

---

## Deploying to hardware (later)

When the arm exists, the only change is the backend:

```python
from loophole_arm.control import make_hardware_robot
robot, home = make_hardware_robot(arm="feetech", port="/dev/ttyUSB0")
robot.backend.connect(); robot.enable()
TrajectoryPlayer(robot).play(Trajectory.load("skills/pick_place.json"))
```

Same skill file, same player, same safety — different backend.
