# CAD import with roles, positioner-axis pick and TCP teach — design

Date: 2026-11-06 · Spec for CP-60 (cell / job import, roles, positioner-axis pick, cross-check) and CP-61 (TCP teach on
the UI), `docs/generated/DES-001_change_proposal.md`. Direction and role list decided by Truong on 2026-11-06; open
values are marked **Open**. Implements DES-001 Rev 0.5 A15, A18 (S2, S10), §5.2, §5.3, §5.4 with the CP-60 / CP-61
changes; keeps CP-58 (seam in the positioner frame) and CP-59 (synchronised 7 axes).

## 1. Problem

- A job created in the browser has no part placement; G1.6 offers a manual pose (Assumption) only.
- The cell positioner axis and the TCP are typed into map files by hand or guessed: the automatic head estimate took
  the cable end as the nozzle (bug of 2026-11-06, CP-61). Every Goal 1 result depended on it.
- The mechanical team already structures the Fusion assemblies by device; the software should use that tree.

## 2. Users and flows

### 2.1 Cell import — Admin, screen S10 "Cell model" (once per cell, again when the cell changes)
1. Upload the cell STEP (robot, tool, positioner, frame); state the up axis of the file.
2. Tree + 3D view. The Admin assigns roles (§3) on tree nodes or by clicking in 3D.
3. **Positioner axis:** click a circular edge (or cylindrical face) of the turntable / shaft → centre + axis from the
   B-Rep, arrow shown, "reverse" button; θ = 0 = positioner pose in the file (decided 2026-11-06). A `REF_POSITIONER`
   marker, if present, pre-fills the pick.
4. **Robot base:** from the placement of the `robot` component (today's `export-scene` rule, ADR-16 Assumption until
   `REF_UR_BASE`); shown as a frame for confirmation.
5. **Tab "Tool / TCP" (CP-61):** click the circular edge of the nozzle mouth → centre + beam axis from the B-Rep, arrow
   out of the nozzle, "reverse"; stick-out mm along the beam (default 0, DES-001 §5.3). tool0 = flange contact rule of
   `export-head` (shown for confirmation).
6. Save → cell model written (frames, head folder, meshes, roles); every job path planned on the old cell is stale.

### 2.2 Job import — Engineer, S1 → S2 (once per part number / revision)
1. S1: upload the job STEP (positioner or its flange + fixture + part; robot / tool optional); up axis.
2. S2 opens with the tree + 3D view of all solids; roles (§3) pre-filled from the saved template when names match.
3. Pick the positioner axis on the job file (same tool as 2.1.3).
4. Cross-check (§5) runs; red blocks seam picking and Plan.
5. Pick seams (edges of `part` solids only, existing G1.2 rules); Plan; S4 as today.

## 3. Roles (decided 2026-11-06)

| UI name | Id | Moves with | Use |
|---|---|---|---|
| Robot | `robot` | robot joints | base location, cross-check; never a collision object (model from `ur_description`) |
| Tool (đầu hàn) | `tool` | flange | head, mount, conduit: display, collision, TCP teach |
| Rotary fixture (gá xoay) | `rotary_fixture` | θ | flange, holders, clamps turning with the part |
| Part (chi tiết hàn) | `part` | θ | seam picking; nozzle tip may touch seam faces only |
| *(unassigned)* | `static` | — | static obstacle (default) |

Explicit "exclude" removes a component from the computation and is listed in the report. A role on an assembly node
applies to its children unless a child has its own. Template: instance name → role, saved per repository
(`cell/roles_template.json`), applied on later imports, always reviewed.

## 4. Data

| File | Schema (new / changed) | Content |
|---|---|---|
| `cell/cad/roles.json`, `jobs/<job>/roles.json` | new `v2/cad_roles` | STEP sha256, up axis, node path → role / exclude, template version, created_by / at |
| `cell/model/frames.yaml` | `awm_cell_frames` (minor) | `world_T_positioner_axis` from the pick: `method: pick`, edge id, circle centre / axis / radius in file mm |
| `cell/model/head/head.yaml` | `awm_head` (minor) | `estimates.tcp.method: teach_ui`, edge id, stick-out, user, date |
| `jobs/<job>/placement.json` (D19) | `v2/placement` (minor) | `source: job_cad`; `positioner_flange_T_part` computed from the job axis pick + tree placements; objects with roles |
| `jobs/<job>/cache/view/view.glb` + `topology.json` (D20) | D20 (minor) | all solids; node extras `role`, `component_path`; `pickable` only for `part` |
| `jobs/<job>/cad_check.json` | new `v2/cad_check` | the checks of §5 with status, values, tolerances |
| `jobs/<job>/seam_axis.json` | `v2/seam_axis` (minor) | inputs add job STEP, `roles.json`, axis pick, cell `frames.yaml` hashes |
| D01 `jobs/<job>/model.step` | — | the job assembly; `job/fixture.step` dropped |

All lengths mm, angles deg (ADR-14); headers with `inputs` SHA-256 (rule 2).

## 5. Cross-check (`cad_check.json`, S2 banner green / yellow / red)

| # | Check | Tolerance | Result |
|---|---|---|---|
| 1 | Job positioner = cell positioner. The job file has its own origin, so only quantities relative to the picked axis are compared: (a) radius of the picked circle, job vs cell; (b) after placing the job axis pick on the cell axis pick, the pose of every `rotary_fixture` component present in both files (matched by instance name) relative to the axis. Catches a wrong pick (other circle, other end of the shaft) and an older / different positioner model | 0.05 mm / 0.1° (decided 2026-11-06, = F8: both sides are nominal CAD, an identical model differs by ~1e-6 mm; real placement error belongs to calibration A52 / A53, not to this check) | red, Plan blocked |
| 2 | Robot in the job file: base → axis transform vs cell | 0.05 mm / 0.1° (as 1) | yellow, cell used |
| 3 | Tool in the job file: TCP vs cell TCP | same as 2 | yellow, info |
| 4 | `rotary_fixture` + `part` around the axis; swept over the θ limits they do not intersect `static` | mesh distance > 0 | red (designed contacts such as bearing seats or belts stay red until the CAD models them as separate components; Truong 2026-11-07) |
| 5 | Picked seam edges belong to `part` solids and lie between two of them | — | refused at pick |
| 6 | Hashes of 4 recorded in `seam_axis.json` | — | stale → "re-plan" |

## 6. Components (one owner each, ADR-15: OCC only in the conda process)

| Piece | Package | Interface |
|---|---|---|
| Tree + roles apply / validate | `awm_geometry` (+ schema in `awm_core`) | CLI `assembly-tree --step` (exists as `inspect-assembly`, JSON out), `apply-roles --step --roles` |
| View model with roles | `awm_geometry` | `export-view` gains `--roles` |
| Circle pick → centre / axis | `awm_geometry` | CLI `pick-circle --step --edge <id>` (edge ids of D20); pure function `circle_frame(edge)` |
| Job placement + cross-check | `awm_geometry` | `job-placement --job --roles --axis-edge` → `placement.json`, `cad_check.json` |
| Cell import | `awm_geometry` | `export-scene` / `export-head` take roles + picks instead of the hand maps (maps stay readable for the spike) |
| TCP teach | `awm_geometry` | `export-head --tcp-edge <id> --stickout-mm v` → head.yaml `teach_ui` |
| API | `awm_api` | `/cell/model` (Admin): upload, tree, roles, picks, save; `/jobs/{id}/roles`, `/jobs/{id}/axis`, `/jobs/{id}/check` (Engineer) |
| Screens | `awm_web` | S10 (tree, roles, axis pick, Tool / TCP tab); S2 (tree panel, roles, axis pick, check banner) |

## 7. Tests that pin it

- Synthetic assembly (OCC): positioner disc + holder + two pipes + a robot block + a tool with a nozzle and a long
  cable behind the flange → roles applied to children; circle pick returns centre / axis ≤ 1e-6 mm / deg; job
  placement equals the known pose; TCP teach on the nozzle mouth equals the known tip; cable end never chosen.
- Cross-check: shifted positioner → red; shifted robot → yellow; part on a static node → red; θ sweep hitting a static
  block → red; unassigned component appears as static in the scene; excluded component listed.
- Golden part: cell import from the cell CAD reproduces today's `frames.yaml` within the pick tolerance; job import
  of `564082` (component tree extracted from the cell CAD) gives J1 r 13.33 mm coaxial; plan J1 + J2 as G1.3.
- API role tests: cell import Admin only; job roles / axis Engineer only; Operator read.

## 8. Out of scope

Collision planning itself (Goal 2), measured calibration (A51–A55), automatic role guessing beyond the name template,
STL import (A16).

## 9. Open

None (all decided 2026-11-06; tolerances of checks 1–2 = 0.05 mm / 0.1°).

Decided 2026-11-06 (Truong): θ = 0 without a marker = positioner pose in the file; test data = job STEP
"positioner + fixture + 564082" extracted by the software from the cell CAD (git-ignored, like every STEP), until the
mechanical team supplies one.
