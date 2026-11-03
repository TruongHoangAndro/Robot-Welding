# S10-lite — cell model role editor: contract v0.1

Status: frozen by the lead on 2026-11-03 for S10-lite (CP-54, pulled forward by Truong). Changes only through the lead.

## Purpose

After a cell / fixture STEP is imported, the person editing the cell sees the component tree in 3D, assigns a role to
every component, optionally gives it a display name, and saves the role map (`cad_map`). The STEP file is never
modified. Scene building (A3 `export-scene`) reads only the saved map. Phase 1 S10-lite has **no server write path**:
the page validates the map and offers it as a file download; the file is put into `cell/cad/cad_map.yaml` (or the
spike record) and committed. Saving through `awm_api` (`/cell/model`, DES-001 §7.4) comes with T3.1.

## Inputs (read-only, served from a folder given by `AWM_VIEW_DIR`, like S2-lite)

| File | Producer | Content |
|---|---|---|
| `cell_components.json` | `awm_geometry.cli inspect-assembly` (A1) | component tree: index, parent, path, instance / product name, placement (mm, file frame), bbox, solids |
| `cell_view.glb` | `awm_geometry.cli export-cell-view` (new, `cad-geometry`) | display meshes, one node per **leaf** component: `name` = `C<index>` (index of `cell_components.json`), `extras` = `{"index": n, "instance_name": "...", "path": "..."}`; file frame, mm, `asset.extras` = `{"unit": "mm", "frame": "file", "schema": "cell_view", "schema_version": "0.1", "up_axis": "+Y"}`; coarse tessellation (linear ≤ 2.0 mm, angular ≤ 30°; changed 2026-11-03 after the first export was 40.8 MB; file size target < 25 MB, raised by the lead 2026-11-03 after the 2.0 mm / 30° export gave 23.0 MB, 36 % of it the LightWELD surface model twice — the nested duplicate is removed in the CAD by OI-23, not in software) — display only |
| `cad_map.yaml` (optional) | previous save | starting point for editing |

## Output

`cad_map.yaml` valid against `src/awm_core/awm_core/schemas/v2/cad_map.schema.json` (example:
`schemas/v2/examples/cad_map.example.json`). Rules the page enforces before it allows the download:

1. **Default deny:** every leaf component is covered by a role, by itself or through an ancestor; uncovered ones are
   listed and highlighted.
2. A component is covered by **one** role only (an ancestor role and a different descendant role is an error, except
   `ignore` on an ancestor with nothing mapped below).
3. Exactly one `frames.ur_base.component`; `frames.positioner_axis` present (point + direction, file frame, mm); the
   page can take the axis from a picked cylindrical component (`main_cylinder` in `cell_components.json`).
4. `roles.part` names exactly the part component(s) of the job; `head_cad` entries name a depth.
5. `display_names` only for existing instance names; never empty strings.

## Roles (fixed list)

`part`, `positioner_rotating`, `cell_static`, `head_cad`, `ignore`; plus the frame picks `ur_base` and
`positioner_axis`. New roles need a contract change.

## Who edits

Decided by Truong, 2026-11-03 (CP-54): **Admin** edits the cell part of the map (robot base, cell static, positioner
axis, permanent rotating set); **Engineer** edits the fixture part per job (fixture, clamps, part-specific holders, the
part). S10-lite has no login yet (no `awm_api`): the page shows which part (cell / job fixture) a role belongs to; the
role check is enforced when saving moves to `awm_api` (T3.1). Splitting the map into a cell file and a job file is a
schema MINOR change made with T3.1; the spike keeps one combined file.
