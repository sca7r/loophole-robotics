# Scene YAML files

Each YAML here describes a complete simulation: which arms, where they sit,
what tables and objects exist, and per-arm safety limits.

Run any of them with:

```bash
loophole-armd --scene examples/scenes/<file>.yaml
```

## Files

| File | What it shows |
| --- | --- |
| `single_arm.yaml` | One arm, one workbench, two cubes — the canonical demo |
| `pickplace_dual.yaml` | Two arms with **different safety envelopes** (arm_b has a tighter workspace and slower velocity cap than arm_a) |
| `handoff_triple.yaml` | Three arms laid out for a hand-off chain, each with its own workspace box |

## Schema

All sections are optional except `arms` (which must have at least one entry).
Open one of the example files for a working starting point.

```yaml
arms:                          # one or more robots
  - name: arm_a                # required, becomes the endpoint name
    kind: feetech              # default: feetech
    mount_pos: [0.0, 0.0, 0.10]  # x, y, z in metres (z above floor)
    safety:                    # optional; omitted → SafetyLimits.feetech_default()
      workspace_min: [-0.05, -0.30, 0.10]
      workspace_max: [ 0.35,  0.30, 0.45]
      max_joint_step: 0.15     # scalar (applies to all 6 joints) or list of 6
      joint_lower: [-3.14, -1.57, -1.57, -1.57, -1.46, -3.14]
      joint_upper: [ 3.14,  1.57,  1.57,  1.57,  1.57,  3.14]
      joint_margin: 0.05

scene:                         # optional
  reference_axes: true         # show RGB axes (red=+X, green=+Y, blue=+Z)
  reference_axes_origin: [0.0, 0.0, 0.10]
  table_grid: true             # 5 cm grid on every table top

  tables:                      # optional list
    - size: [0.35, 0.45]       # half-extents in metres
      height: 0.10             # top-surface height above floor
      pos: [0.0, 0.0]          # centre on floor
      name: table_a            # optional

  objects:                     # optional list — physically simulated free bodies
    - kind: cube               # cube | sphere | cylinder
      size: 0.025              # cube/sphere: scalar; cylinder: [radius, half_height]
      pos: [0.18, 0.08, 0.13]
      color: orange            # named, or [r,g,b] / [r,g,b,a]
      mass: 0.05               # optional, default 0.05 kg
```

Named colors: `orange`, `red`, `blue`, `green`, `yellow`, `white`, `black`,
`grey`, `purple`. Or pass an explicit `[r, g, b]` / `[r, g, b, a]` tuple.

## Connecting clients

Once the server is up, any client connects to a named arm:

```bash
loophole-arm-teleop arm_a               # numpad teleop
loophole-arm-teach connect arm_b        # teach over the wire
```

Each terminal can drive a different arm independently.
