# UR5e (reference robot)

A Universal Robots UR5e with a Robotiq 2F-85 gripper, used as the reference
industrial arm. The model files are vendored from MuJoCo Menagerie
(Apache-2.0) and are NOT stored in this folder; run
`bash scripts/fetch_menagerie.sh` to download them into `assets/menagerie/`.

`robot.yaml` in this folder is the catalog entry: joints, home pose, and
where the vendored model lives.
