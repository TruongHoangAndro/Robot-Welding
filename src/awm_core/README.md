# awm_core (M0)

Pure Python library shared by every APLSMN601 package. Importable from the ROS 2 Jazzy
interpreter (3.12) and from the OCC conda interpreter `awm_occ` (3.10): numpy + jsonschema only.

| Module | Content |
|---|---|
| `jsonio` | Schema-validated read / write, common header (`schema_version`, `created_by`, `created_at`, `inputs` with SHA-256), stale-input detection |
| `schemas/*.schema.json` | D03 part, D04 seams, D05 wps, D06 process, D07 path, D08 calibration, D09 program, D10 release, D15 verify_report, A72/A73 record, derived edges / seam_geometry |
| `frames` | Quaternions (x, y, z, w), RPY, 4x4 transforms, `frame_from_z_x` |
| `calibration` | Active calibration (D08) and `base_T_part = base_T_fixture · fixture_T_part` |
| `repository` | Job repository layout (DES-001 §5.2): `Repository`, `JobPaths`, program revisions `P###` |
| `occ_bridge` | Runs `awm_geometry.cli` in the OCC interpreter (`occ_python_cmd` / `AWM_OCC_PYTHON`) |
| `stl` | Minimal STL reader for planning-scene meshes |

CLI: `ros2 run awm_core awm_repo init --repo <root> --template <dir with cell/>`.

Tests: `scripts/dx "colcon test --packages-select awm_core"` (also pass with
`$AWM_OCC_PYTHON -m pytest src/awm_core/test`).
