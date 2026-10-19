# awm_bringup

```bash
ros2 launch awm_bringup sim.launch.py [headless:=true]   # mock hardware + move_group (+ RViz)
```
Mock hardware only. The `real` and `workbench` launches of DES-001 are not part of this delivery;
a real-robot launch is run by a person only.

| Path | Content |
|---|---|
| `config/awm_params.yaml` | D16: the single parameter file of the chain (repo root, OCC interpreter, groups, planning, timing, verification) |
| `repo_template/cell/` | SAMPLE WPS (D05), NOMINAL calibration (D08, `active.json`), io_map placeholder (D11) |
| `awm_bringup.testing.SimLaunch` | runs the headless sim in a private ROS domain (tests, `run_chain --start-sim`) |
