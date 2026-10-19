# awm_geometry (M1/M2 OCCT helper, ADR-02)

All B-Rep work: STEP load, check & heal, tessellation, edge catalogue, seam sampling.
Runs only in the conda env `awm_occ` (pythonocc-core 8.0.1 / OCCT 8.0.1). No rclpy. ROS packages
call it through `awm_core.occ_bridge` (subprocess).

```bash
PYTHONPATH=src/awm_geometry:src/awm_core $AWM_OCC_PYTHON -m awm_geometry.cli <command>
  info
  import --job <job_dir> --step part.step --part-no N --part-rev R   # A11-A14
  sample --job <job_dir> --spacing-m 0.005                           # A41 input
  make-test-part --kind plate_tjoint|pipe_flange --out x.step --picks x.json
```

Outputs (part frame, metres): `model.step` (read-only copy), `cache/model.brep`, `cache/mesh.stl`,
`cache/edges.json` (edge ids, convexity from `BRepOffset_Analyse`, polylines, fingerprints),
`part.json` (D03), `cache/seam_geometry.json` (points, tangents, face normals).

Notes: STEP is read in OCCT's millimetres and scaled to metres explicitly; edge ids are the indices
of `TopTools_IndexedMapOfShape` on the healed shape. OCC 8 API: `topods.Edge`, `topexp.MapShapes`,
`brepbndlib.Add`, no `TopoDS_Shape.HashCode`.

Tests: `$AWM_OCC_PYTHON -m pytest src/awm_geometry/test`; `colcon test` runs the same suite in the
conda interpreter (`test_occ_suite_in_conda.py`) and the subprocess path.
