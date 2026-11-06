# awm_program (M6, A61-A64)

`path.json` + active calibration -> `programs/P###/program.json` (D09): air moves (OMPL, densely
collision-checked), approach / weld / retract segments from the checked joint path (verified by FK
against the transformed seam, A61), weld timing at the WPS travel speed (A62, constant TCP speed),
WeldStart after the approach segment and WeldEnd after the weld segment, both at rest (A63),
storage per revision with input hashes and the calibration hash (A64).

```bash
ros2 run awm_program generate --job <job> --ros-args --params-file .../awm_params.yaml
```
Refuses: stale inputs, other calibration than the path, rejected seams (no partial programs).

## G1.5: synchronised 7-joint program (D09 v2 MINOR 2.1)

```bash
python3 -m awm_program.cli generate --job <job_dir> [--path <path.json>] [--head-dir <dir>]
```
From the G1.3 kinematic path D07 (`collision_checked: false`): one `robot_positioner` trajectory per segment
(air / index joint-interpolated and not collision-checked, approach 40 mm, weld at the travel speed relative to the
part with a ramp on the seam, retract 40 mm), WeldStart / WeldEnd only with robot and positioner at rest, events
without process data (no D06 yet). Refuses stale D07, rejected arcs, travel speeds above the feasible maximum
(number shown), another description or calibration. Pure core in `awm_program/core/` (no rclpy).
