# Feetech arm

The first Helix robot: a 6-DOF arm built from Feetech STS3215 serial-bus
servos, with a single-DOF jaw gripper (one hinge drives the linkage; the
visible finger plates are not separate axes).

Files in this folder:

| File | What it is |
| --- | --- |
| `robot.yaml` | Single source of truth: joints, home pose, gripper, motor channels |
| `arm_mujoco.urdf` | The URDF the simulator and IK load (MuJoCo-compatible) |
| `arm_description.urdf` | Original CAD-exported URDF, kept for reference |
| `meshes/` | STL meshes referenced by the URDFs |

To use a different arm, copy this folder, edit `robot.yaml` and the model
files, and load it by name: `load_robot("<folder_name>")`. Nothing outside
the folder needs to change.
