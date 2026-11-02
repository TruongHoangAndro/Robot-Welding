# D20 — 3D view model contract (v2.0)

Status: frozen by the lead on 2026-11-02 for the D20 preview (part of PLAN.md T0.5). Changes only through the lead.
Design source: DES-001 Rev 0.5 §4.5 (3D view model), §5.1 (D20), CP-52 (joints between separate solids).

Producer: `awm_geometry` (M1, conda OCC process). Consumers: `awm_web` (S2 seam picker, S4), `awm_seam` (M2).
Location in the job package: `jobs/<PartNo>_<PartRev>/cache/view/{view.glb, topology.json}`.

## Rules

1. **Display only.** The browser picks ids; M2 resolves ids against the exact B-Rep. Display geometry is never a
   weld path.
2. **Frame and units.** Part frame, **mm**, as read from the source STEP (no rotation applied). `topology.json`
   states the source up axis; the viewer rotates to +Z up for display only. glTF nominally uses metres — this file
   deliberately stores mm and says so in `asset.extras.unit = "mm"`; consumers must not rescale it.
3. **Ids.** `F<n>`, `E<n>`, `B<n>` are 1-based indices of `TopTools_IndexedMapOfShape` (`topexp.MapShapes`) on the
   healed part shape (compound of its solids), the same scheme as the existing edge catalogue (`awm_geometry.topology`).
   Same STEP + same code → same ids (tested). Joint edges `JE<n>` are numbered in the order of
   `awm_geometry.joints.find_joint_edges`. Every edge of a joint carries its `joint_edge_id`; picking either edge
   selects the joint.
4. **Header** as every job file: `schema_version` "2.0", `created_by`, `created_at`, `inputs` with SHA-256 of the
   STEP file.

## view.glb (glTF 2.0 binary)

| Element | Convention |
|---|---|
| `asset.extras` | `{"unit": "mm", "frame": "part", "schema": "D20", "schema_version": "2.0"}` |
| Scene | one root node `part`; children: one node per solid `B<n>` |
| Face | child node of its solid, `name` = `F<n>`, `extras` = `{"face_id": "F<n>", "solid_id": "B<n>"}`; one mesh, one primitive, mode 4 (TRIANGLES), attributes `POSITION`, `NORMAL`, uint32 indices |
| Edge | child node of its solid, `name` = `E<n>`, `extras` = `{"edge_id": "E<n>", "solid_id": "B<n>", "joint_edge_id": "JE<n>" or null}`; one primitive, mode 3 (LINE_STRIP), attribute `POSITION`; chord ≤ `edge_chord_mm` (≤ 0.05 mm) |
| Materials | none required; the viewer colours by role |
| Precision | float32 positions; no Draco / meshopt compression (no extensions) |

## topology.json

Schema `src/awm_core/awm_core/schemas/view_topology.schema.json`; example
`src/awm_core/awm_core/schemas/examples/view_topology.example.json`. Contents: source (file, component, up axis),
tessellation tolerances, solids, faces, edges (curve type, length, closed, adjacent faces, G1 neighbours, fingerprint,
joint edge id) and joints (solid pair, edge pair, length, closed, 4 preview seam frames with X = tangent and
Z = bisector of the exterior-face normals out of the material, the rule of `awm_geometry.joints`).

## CLI

`$AWM_OCC_PYTHON -m awm_geometry.cli export-view --step <file> [--component <name>] [--up-axis +Y] --out <dir>`
writes `<dir>/view.glb` and `<dir>/topology.json`. Component names and the up axis are arguments, never constants
in source.
