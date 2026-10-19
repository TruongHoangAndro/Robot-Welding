# awm_cad (M1, A11-A14)

ROS-side wrapper of the STEP import: runs `awm_geometry import` in the OCC interpreter and
validates `part.json` / `edges.json`.

```bash
ros2 run awm_cad import_step --repo <repo_root> --step part.step --part-no 1234 --part-rev B
```
Python API: `awm_cad.import_step(repo_root, step, part_no, part_rev, occ_python_cmd=None)`.
