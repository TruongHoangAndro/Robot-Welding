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
