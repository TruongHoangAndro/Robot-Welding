# awm_path (M4, A41-A45)

Seam samples -> torch poses (A42: bisector or work angle from a reference face, travel angle,
`tool_roll`), approach / retract along the torch axis (A43), IK branch search (pick_ik global) and
100 % collision-checked Cartesian approach / weld / retract (KDL) against the cell model and the
part mesh (A44, A45). Writes `path.json` (D07, part frame) with the checked joint path; seams that
fail are listed in `rejected` with the reason. No partial paths, no fallbacks.

```bash
ros2 run awm_path plan_path --job <job> --ros-args --params-file \
    $(ros2 pkg prefix awm_bringup)/share/awm_bringup/config/awm_params.yaml
```
Needs `sim.launch.py` (move_group). Exit 0 all accepted, 2 some rejected, 1 error.
