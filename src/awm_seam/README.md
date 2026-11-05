# awm_seam (M2, A21 / A22 / A24)

## v2 — joint seams and the seam in the positioner frame (G1.2, PLAN T2.2, CP-52 / CP-53 / CP-58 / CP-59)

A picked edge id (`E<n>` of either solid, from D20 `cache/view/topology.json`) names the joint between two solids
(`joint_edge_id`). The seam is that joint, sampled on the exact B-Rep in the conda CAD process
(`awm_geometry.cli sample-joints`, called through `awm_core.occ_bridge`) at a pitch ≤ 1 mm with the joint-aware
normals n1 / n2. Outputs (mm, degrees):

- `seams.json` (schema `v2/seams`, 2.1): attributes entered by the Engineer — seam id, direction (`reversed`),
  start (`start_offset_mm` or a start point), `arcs`, `arc_overlap_mm`, `positioner_mode` (`sync` default | `C` | `B`),
  `positioner_angles_deg` (one per arc, mode B only), joint type, sequence — plus the samples in the `part` frame.
- `seam_axis.json` (schema `v2/seam_axis`): every sample and its seam frame in `positioner_flange` at θ = 0, with
  `r_mm`, `axial_mm`, `phi_deg`, arcs and a coaxial test; inputs = SHA-256 of `seams.json`, `placement.json`,
  `../../cell/model/frames.yaml`. Readers (`seam_axis.read_seam_axis`) refuse a stale file.

```bash
ros2 run awm_seam joint_seam build --job <job_dir> --picks picks.json [--pitch-mm 1.0]   # AWM_OCC_PYTHON set
ros2 run awm_seam joint_seam axis  --job <job_dir>     # after placement.json / frames.yaml changed
ros2 run awm_seam joint_seam show  --job <job_dir>
```
Pick file format: `python3 -m awm_seam.joint_cli --help`.

## v1 — edge catalogue seams (metres, kept until T0.1)

`ros2 run awm_seam seam define|from-picks|list|remove|rematch ...` on `cache/edges.json`, see `awm_seam/cli.py`.
