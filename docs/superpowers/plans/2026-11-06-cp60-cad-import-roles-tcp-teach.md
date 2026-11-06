# CP-60 / CP-61 — CAD import with roles, positioner-axis pick, cross-check, TCP teach — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Admin imports the cell STEP once (roles, positioner-axis pick, TCP teach); the Engineer imports a job
STEP (positioner / fixture / part), assigns roles, picks the positioner axis, gets a green cross-check and plans — with
no hand-typed map, pose or TCP.

**Architecture:** Roles live in a new `roles.json` (`v2/cad_roles`); pure role logic (tree paths, inheritance,
validation, template) in `awm_core.cad_roles`, used by `awm_geometry`, `awm_seam` and `awm_api`. The cell import
compiles `roles.json` + the axis pick into the existing `cad_map/0.1` dict, so `export-scene` keeps one code path.
New OCC work (circle pick, job placement, cross-check, TCP teach) stays in `awm_geometry` CLI commands (ADR-15);
`awm_api` runs them as runs; `awm_web` gets S10 (Admin) and S2 additions (Engineer).

**Tech Stack:** pythonocc 8.0.1 (conda `awm_occ`), numpy, jsonschema, FastAPI, React + TypeScript + r3f.

**Spec:** `docs/superpowers/specs/2026-11-06-cad-import-roles-tcp-teach-design.md` (CP-60, CP-61 in
`docs/generated/DES-001_change_proposal.md`). Supersedes the unexecuted `2026-11-05-wp3-s10-lite-editor.md`
(its 5 roles are replaced by the 4 roles of CP-60).

## Global Constraints

- Simulation only; never connect to a robot, PLC, LightWELD or laser; no real IPs; no speed / acceleration limit raised.
- OCC only in the conda process (`$AWM_OCC_PYTHON -m awm_geometry.cli …`); never import OCC in `awm_core`, `awm_seam`,
  `awm_api` or any ROS node.
- Units: mm / deg / s in every file and message; m ↔ mm only via `awm_core.units`.
- Every JSON written has `schema_version`, `created_by`, `created_at`, `inputs` (SHA-256), validated by its schema.
- No part data, component names or poses in source; golden values only in tests gated by `AWM_CELL_STEP`.
- STEP files are inputs, never committed (also the extracted job STEP).
- Roles: `robot`, `tool`, `rotary_fixture`, `part`; unassigned = `static`; `exclude` is an action, not a role.
- A role on a node applies to its descendants unless a descendant has its own.
- Cross-check tolerances (checks 1–3): **0.05 mm and 0.1°**. Check 1 red blocks seam picking and Plan; checks 2–3
  yellow; check 4 red; check 5 refused at pick; check 6 stale → re-plan.
- θ = 0 = positioner pose in the file when there is no `REF_POSITIONER` marker.
- Positioner-axis frame (cell and job, one function): origin = picked circle centre, x = axis (after "reverse"),
  z = world up (file up axis rotated to +Z) made orthogonal to x; vertical axis refused.
- `docs/reference/**`, `docs/LLM_Wiki/**` read-only. ROS / pytest-with-ROS through `scripts/dx`.
- Commit prefixes `<package>:`; end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Never push or merge.

## Review Focus

1. Job file with the positioner at another angle than the cell file → check 1 must stay green and the placement must
   compensate the angle φ about the axis (Task 5 `test_job_positioner_rotated_30deg`).
2. Job file exported with another origin / up axis → identical placement (Task 5 `test_job_file_moved_and_y_up`).
3. A picked edge that is a line, B-spline or ellipse → refused with "not a circle"; an arc of a circle is accepted
   (Task 3 `test_pick_refuses_non_circle`, `test_pick_accepts_arc`).
4. Roles or axis edited after planning → `seam_axis.json` and the path become stale, Plan re-runs
   (Task 7 `test_roles_edit_makes_seam_axis_stale`).
5. Same instance name under two parents → roles by path are fine; check 1b refuses the ambiguous match
   (Task 5 `test_duplicate_instance_name_is_red`; Task 1 `test_paths_distinguish_duplicate_names`).

## Owners and order (≤ 2 agents at a time)

| Branch | Agent | Tasks | Starts |
|---|---|---|---|
| `cp60-geometry` | cad-geometry | 1–6 | now |
| `cp60-web` | web | 8–11 (API first against `fake_cli`, then UI) | after Task 1 is committed (schemas + `cad_roles` frozen) |
| `cp60-seam` | seam-process | 7 | after Tasks 1–3 merged |
| GotoCode | lead | 12 | after all merged |

---

### Task 1: Schemas and pure role logic (`awm_core`)

**Files:**
- Create: `src/awm_core/awm_core/cad_roles.py`, `src/awm_core/awm_core/axis_frame.py`
- Create: `src/awm_core/awm_core/schemas/v2/cad_roles.schema.json`, `v2/axis_pick.schema.json`, `v2/cad_check.schema.json` (+ one example each in `v2/examples/`)
- Modify: `v2/placement.schema.json` (`source` enum adds `job_cad`; optional `axis_pick`, `phi_deg`), `view_topology.schema.json` (solids optional `component_index`, `component_path`; `source.components` bool; face / edge `solid_id` may be null), `v2/cad_map.schema.json` (`frames.positioner_axis.pick` optional object = axis_pick body), `v2/seam_axis.schema.json` (no shape change; doc note on inputs)
- Test: `src/awm_core/test/test_cad_roles.py`, `src/awm_core/test/test_axis_frame.py`, `src/awm_core/test/test_schemas_cp60.py`

**Interfaces (Produces):**
- `ROLES = ('robot', 'tool', 'rotary_fixture', 'part', 'static')`, `EXCLUDE = 'exclude'`
- `rel_paths(tree: list[dict]) -> list[str]` — moved from `awm_geometry.scene._rel_paths` (same output; scene imports it).
- `effective_roles(tree, assignments: list[dict]) -> dict[int, str]` — every node index → role or `exclude`; own
  assignment, else parent's, else `static`.
- `validate_roles(tree, doc: dict, scope: Literal['cell', 'job']) -> list[str]` — problems (empty = ok):
  unknown path, path assigned twice, unknown role; cell: exactly one top-most `robot` node, ≥ 1 `rotary_fixture`,
  no `part`; job: ≥ 1 `part` leaf with geometry, ≥ 1 `rotary_fixture`, ≤ 1 `robot` node.
- `robot_node(tree, eff) -> int | None`, `nodes_with_role(tree, eff, role) -> list[int]` (top-most nodes of that role).
- `apply_template(tree, template: dict) -> list[dict]` (instance name → role, only names that occur once),
  `update_template(template, tree, assignments) -> dict`.
- `from_cad_map(tree, cmap: dict) -> list[dict]` — old map → assignments (`positioner_rotating`→`rotary_fixture`,
  `cell_static`→`static`, `head_cad`→`tool`, `ignore`→`exclude`, `frames.ur_base.component`→`robot`, `part`→`part`).
- `axis_frame.file_T_axis(center_file_mm, axis_file, up_axis: str) -> np.ndarray (4x4)` and
  `axis_frame.world_R_file(up_axis) -> np.ndarray` (moved from `scene.world_R_file`; scene re-exports).
- roles.json: `{header…, source: {file, sha256, up_axis}, scope, assignments: [{path, role}], template_version}`.
- axis_pick.json: `{header…, source: {file, sha256}, component_path|null, edge_id, center_file_mm[3], axis_file[3],
  radius_mm, reversed, method: "pick"}`.
- cad_check.json: `{header…, status: green|yellow|red, checks: [{id: 1..6, name, status, values, tolerance, note}],
  phi_deg}`.

- [ ] **Step 1: Write the failing tests**

```python
def test_child_role_overrides_parent(tree_small):        # A(rotary) / B(part) / C(none)
    eff = effective_roles(tree_small, [{'path': 'A', 'role': 'rotary_fixture'}, {'path': 'A / B', 'role': 'part'}])
    assert eff[idx('A / B')] == 'part' and eff[idx('A / C')] == 'rotary_fixture' and eff[idx('D')] == 'static'

def test_paths_distinguish_duplicate_names(tree_dup):    # 'Holder:1' under X and under Y
    assert {'X / Holder:1', 'Y / Holder:1'} <= set(rel_paths(tree_dup))

def test_cell_scope_needs_one_robot_and_no_part(tree_small):
    assert any('robot' in p for p in validate_roles(tree_small, doc([]), 'cell'))
    assert any('part' in p for p in validate_roles(tree_small, doc([('R', 'robot'), ('A', 'rotary_fixture'), ('A / B', 'part')]), 'cell'))

def test_from_cad_map_matches_old_roles(tree_small, cmap_small):
    eff = effective_roles(tree_small, from_cad_map(tree_small, cmap_small))
    assert eff == expected_from_old_map

def test_axis_frame_matches_scene_rule():                 # +Y-up file, axis file +X through (1, 2, 3)
    T = file_T_axis([1, 2, 3], [1, 0, 0], '+Y')
    np.testing.assert_allclose(T[:3, 3], [1, 2, 3]); np.testing.assert_allclose(T[:3, 0], [1, 0, 0])
    np.testing.assert_allclose(world_R_file('+Y') @ T[:3, 2], [0, 0, 1], atol=1e-12)

def test_axis_frame_vertical_refused():
    with pytest.raises(ValueError, match='vertical'): file_T_axis([0, 0, 0], [0, 1, 0], '+Y')
```
Plus `test_schemas_cp60.py`: each new example validates; `placement` with `source: job_cad` validates; a D20 topology
with `solid_id: null` faces validates.

- [ ] **Step 2: Run, expect FAIL** — `scripts/dx "python3 -m pytest src/awm_core/test/test_cad_roles.py src/awm_core/test/test_axis_frame.py src/awm_core/test/test_schemas_cp60.py -v"` → ImportError.
- [ ] **Step 3: Implement** the interfaces above. `file_T_axis` = the rule now inside `scene.cell_frames` (x = axis,
  z = up ⟂ x, y = z × x, expressed in the file frame).
- [ ] **Step 4: Refactor** `scene.cell_frames` / `_rel_paths` / `world_R_file` to use the `awm_core` functions;
  run `$AWM_OCC_PYTHON -m pytest src/awm_geometry/test -q` (conda, needs `PYTHONPATH=src/awm_geometry:src/awm_core`) → no regression; Step 2 command → PASS.
- [ ] **Step 5: Commit** `awm_core: CP-60 roles, axis frame, cad_roles / axis_pick / cad_check schemas`.

---

### Task 2: Assembly-aware loader and D20 with components (`awm_geometry`)

**Files:** Modify `awm_geometry/part.py`, `assembly.py`, `view.py`, `joint_seam.py`, `cli.py`; extend `testparts.py`
(kind `job_assembly`: positioner disc + holder + two coaxial pipes + robot block + tool with nozzle and cable behind the
flange, instance names unique except one duplicated holder name under two parents; optional `--transform`).
Test: `src/awm_geometry/test/test_assembly_view_occ.py`.

**Interfaces:**
- Consumes: Task 1 `rel_paths`.
- Produces: `part.load_part(step, component=None, components=False) -> LoadedPart`; with `components=True` the whole
  file is read through XCAF and `LoadedPart.solid_components: list[int | None]` gives the `inspect-assembly` index per
  solid. `export-view --components` writes topology `source.components: true` and per solid `component_index`,
  `component_path`; faces of shells without a solid get `solid_id: null`. `sample-joints` loads the model with the
  same flag read from `topology.source.components`.

- [ ] **Step 1: Failing tests**

```python
def test_solids_carry_component_paths(job_assembly_step, tmp_path):
    topo = export_view(job_assembly_step, components=True, out=tmp_path)
    assert {s['component_path'] for s in topo['solids']} >= {'Positioner / Disc:1', 'Fixture / Holder:1', 'Part / PipeA:1'}

def test_edge_ids_same_in_view_and_sampler(job_assembly_step, tmp_path):
    # the joint ring between the pipes has the same E-id in export-view and in sample-joints
    assert sampled_edge_ids(...) == view_joint_edge_ids(...)

def test_shell_faces_exported_with_null_solid(tool_shell_step, tmp_path):
    topo = export_view(tool_shell_step, components=True, out=tmp_path)
    assert any(f['solid_id'] is None for f in topo['faces']) and topo['edges']
```
- [ ] **Step 2: Run** `$AWM_OCC_PYTHON -m pytest src/awm_geometry/test/test_assembly_view_occ.py -v` → FAIL.
- [ ] **Step 3: Implement**; the old whole-file path (`components=False`) stays byte-identical (existing tests unchanged).
- [ ] **Step 4: Run** the new file and the full conda suite → PASS.
- [ ] **Step 5: Commit** `awm_geometry: assembly-aware loader, D20 solids with component paths`.

---

### Task 3: Circle pick (`pick-circle`)

**Files:** Create `awm_geometry/circle_pick.py`; modify `cli.py`. Test `test_circle_pick_occ.py`.

**Interfaces:**
- Produces: `circle_frame(edge: TopoDS_Edge) -> dict` (`center_mm`, `axis`, `radius_mm`; raises
  `GeometryError('edge E<n> is not a circle (<type>)')` for non-circles); CLI
  `pick-circle --step F [--component NAME | --components] --edge E<n> [--reverse] --up-axis A --out axis_pick.json`
  (loads with exactly the arguments of the view the edge id came from; writes `v2/axis_pick`).

- [ ] **Step 1: Failing tests**

```python
def test_pick_ring_centre_and_axis(job_assembly_step):     # disc rim: centre (0, -450, 150), axis +X, r 60
    d = pick(job_assembly_step, edge=disc_rim_edge_id)
    np.testing.assert_allclose(d['center_file_mm'], [0, -450, 150], atol=1e-6)
    assert angle_deg(d['axis_file'], [1, 0, 0]) < 1e-6 and abs(d['radius_mm'] - 60.0) < 1e-6

def test_pick_reverse_flips_axis(...): assert reversed axis == -axis, centre unchanged
def test_pick_refuses_non_circle(...): line edge and B-spline edge → GeometryError 'not a circle'
def test_pick_accepts_arc(...): half-circle edge → same centre / axis / radius as the full circle
```
- [ ] **Step 2–4:** run → FAIL, implement with `BRepAdaptor_Curve.GetType() == GeomAbs_Circle` and `.Circle()`,
  run → PASS.
- [ ] **Step 5: Commit** `awm_geometry: pick-circle (circle centre / axis from a D20 edge id)`.

---

### Task 4: Cell import from roles (`apply-roles`, cell-only `export-scene`)

**Files:** Create `awm_geometry/roles_map.py`; modify `scene.py`, `cli.py`. Test `test_roles_map.py` (pure) and
`test_cell_import_occ.py`.

**Interfaces:**
- Consumes: Task 1 (`validate_roles`, `effective_roles`, `robot_node`), Task 3 axis_pick.json.
- Produces: `compile_cad_map(tree, roles_doc, axis_pick) -> dict` (valid `cad_map/0.1`: one entry per top-most
  uniform-role subtree as `{path: …}`, `robot` subtree → `ignore` + `frames.ur_base.component`, `rotary_fixture` →
  `positioner_rotating`, `static` → `cell_static`, `tool` → `head_cad`, `exclude` → `ignore`;
  `frames.positioner_axis` = pick centre / axis + `pick` block); CLI
  `apply-roles --step F --tree cell_components.json --roles roles.json --axis axis_pick.json --out cad_map.yaml`;
  `export-scene --job` becomes optional: without it only `cell/model` is written (no part import).
  `frames.yaml` `world_T_positioner_axis` gains `method: pick`, `edge_id`, `radius_mm` when the map has a pick.

- [ ] **Step 1: Failing tests**

```python
def test_compiled_map_has_no_nested_entries(tree_small, roles_cell, pick):
    cmap = compile_cad_map(tree_small, roles_cell, pick)
    scene.resolve_roles(tree_small, cmap)            # does not raise (no role inside another role)

def test_unassigned_component_becomes_static(...): assert 'D' in [e['path'] for e in cmap['roles']['cell_static']]
def test_excluded_component_listed(...): assert summary['excluded'] == ['E']

def test_cell_only_export_writes_frames_with_pick(job_assembly_step, tmp_path):   # OCC
    run apply-roles + export-scene without --job
    fr = miniyaml.load(tmp_path / 'cell/model/frames.yaml')
    assert fr['world_T_positioner_axis']['method'] == 'pick' and not (tmp_path / 'jobs').exists()
```
Golden (skipped without `AWM_CELL_STEP`): roles from `from_cad_map(cad_map_cell.yaml)` with the `564082` component set
to `exclude`, axis = circle edge of `Part1:1` found by the test (axis ∥ file +X within 0.1°, on the line of
`cad_map_cell.yaml` `positioner_axis` within 0.05 mm, largest radius) → new `frames.yaml` axis line equals the current
one within 0.05 mm / 0.1° and `world_T_base` is unchanged within 1e-6 mm.
- [ ] **Step 2–4:** run → FAIL, implement, run new + full conda suite → PASS; run the golden with `AWM_CELL_STEP` (0 skipped).
- [ ] **Step 5: Commit** `awm_geometry: apply-roles compiles roles + axis pick into the cell map; cell-only export-scene`.

---

### Task 5: Job placement and cross-check (`job-placement`)

**Files:** Create `awm_geometry/job_placement.py`, `awm_geometry/cross_check.py`; modify `cli.py`, `testparts.py`
(`make-job-step` test-data command, see Step 6). Test `test_cross_check.py` (pure numpy) and `test_job_placement_occ.py`.

**Interfaces:**
- Consumes: Tasks 1–3; cell `frames.yaml`, `cell/cad/roles.json`, `cell/cad/axis_pick.json`,
  `cell/cad/cell_components.json`, cell scene manifest (static objects + meshes).
- Produces: CLI `job-placement --repo R --job J` → `jobs/J/placement.json` (`source: job_cad`,
  `flange_T_part = Rx(φ) · inv(file_T_axis_job)`, objects: `part` mesh `cache/collision.stl` with the **D20 face ids**,
  job `rotary_fixture` components not matched in the cell as hulls, parent `positioner_flange`), `jobs/J/cad_check.json`;
  exit 0 also when red (the check is the result); exit 1 only on bad inputs.
- `cross_check.positioner_match(cell, job, tol_mm=0.05, tol_deg=0.1) -> CheckResult` with `phi_deg`;
  `cross_check.swept_envelope_hits(rotating_pts_axis, static_pts_axis, slab_mm=1.0) -> list[hit]`.

Rules (from the spec, fixed here):
- Check 1: (a) |r_job − r_cell| ≤ 0.05 mm; (b) for every `rotary_fixture` instance name present once in both files,
  `Δ_c = axis_T_c(cell) · inv(axis_T_c(job))`; φ = rotation of Δ for the first match about x; each Δ_c · Rx(−φ) must be
  within 0.05 mm / 0.1° of identity. No common name → red ("positioner cannot be confirmed"); a name twice in a file → red.
- Check 2 (robot node in the job file): `axis_T_robot` job vs cell within 0.05 mm / 0.1° → else yellow; absent → skip.
- Check 3 (tool node in both files): `robot_T_tool` job vs cell, same tolerance → yellow; absent → skip.
- Check 4: rotating set = job `rotary_fixture` + `part` surfaces sampled ≤ 1 mm, in the axis frame; per 1 mm slab along
  x and per component an interval [r_min, r_max]; cell `static` object surfaces sampled ≤ 1 mm; a static sample inside
  an interval of its slab = hit → red, listing object, slab and depth. Full turn (angle limits unknown, OI-27: conservative).
- Check 5 is enforced in Task 7; check 6 = input hashes, recorded here in `cad_check.json` `inputs`.

- [ ] **Step 1: Failing tests (pure)**

```python
def test_phi_recovered_from_rotated_job():          # job components = cell components rotated 30 deg about x
    r = positioner_match(cell, rotated(job, 30.0)); assert r.status == 'green' and abs(r.phi_deg - 30.0) < 1e-9
def test_shifted_along_axis_is_red():               # other end of the shaft: 120 mm along x
    assert positioner_match(cell, shifted(job, [120, 0, 0])).status == 'red'
def test_radius_off_by_0p06_is_red(): ...
def test_duplicate_instance_name_is_red(): ...
def test_static_inside_swept_ring_hits(): one static point at (a=5, r=50) vs ring interval [40, 60] → 1 hit
def test_static_in_bore_does_not_hit(): static shaft r=10 vs ring interval [40, 60] → no hit
```
- [ ] **Step 2–4:** run `scripts/dx "python3 -m pytest …/test_cross_check.py -v"` → FAIL, implement, PASS.
- [ ] **Step 5: OCC tests** (`test_job_placement_occ.py`, synthetic `job_assembly` with a cell built by Task 4):
  `test_job_positioner_rotated_30deg` (green, φ 30, the ring seam lands on the same world points within 1e-6 mm),
  `test_job_file_moved_and_y_up` (`make-job-step --transform` + `+Y` up → same `flange_T_part` within 1e-6),
  `test_static_block_in_swept_volume_is_red`, `test_collision_mesh_keeps_d20_face_ids` (every triangle centroid lies on
  its D20 face within 0.01 mm), `test_unassigned_appears_static` and `test_excluded_listed` (cad_check notes).
- [ ] **Step 6: Test data tool** `make-job-step --step CELL --components NAME… --out F.step [--transform x y z rx ry rz]`
  (XCAF copy keeping instance names and placements; test / demo data, output git-ignored). Golden (needs
  `AWM_CELL_STEP`): extract `Part1:1, Part2:1, Part3:1, HTPK20S5M150-A-C15:1, HTPK20S5M150-A-C15:4,
  564082-Assembly-3 parts:2` → job import → check green, J1 (r 13.33 mm) coaxial with the cell axis within 0.05 mm /
  0.1°. If check 4 is red on the golden, stop and report the object to the lead (no tolerance change without Truong).
- [ ] **Step 7: Commit** `awm_geometry: job-placement with cross-check (CP-60 checks 1-4), make-job-step test data`.

---

### Task 6: TCP teach (`export-head --tcp-edge`)

**Files:** Modify `awm_geometry/head.py`, `cli.py`. Test `test_head_teach_occ.py`.

Scope decision: the Tool / TCP tab works on the head STEP and head map in `cell/cad/` (`head.step`,
`cad_map_head.yaml`): the tool0 rule needs the wrist-3 flange, which the cell CAD nests inside the robot; the head link
split (body / mount) stays in the head map until a sub-role is decided.

**Interfaces:** `export-head --step F --map M --repo R --tcp-component NAME --tcp-edge E<n> [--tcp-reverse]
[--stickout-mm 0] [--taught-by USER]`. Beam axis sign: away from the `head_body` centroid, flipped by `--tcp-reverse`;
TCP = circle centre + stick-out · axis; then the existing map-override path (`beam_exit = tcp − nozzle_len · axis`).
head.yaml `estimates.tcp = {method: teach_ui, edge_id, component, stickout_mm, taught_by, taught_at}`. The guard
"TCP behind the flange → refused" stays.

- [ ] **Step 1: Failing tests** (synthetic tool with nozzle mouth circle at known tip, cable end behind the flange):
  `test_teach_tip_equals_known_tip` (≤ 1e-6 mm, axis ≤ 1e-6°), `test_stickout_moves_tcp_along_beam` (5 mm),
  `test_cable_end_edge_refused` (TCP behind the flange → error), `test_head_yaml_records_teach_ui`.
- [ ] **Step 2–4:** FAIL → implement → PASS (+ full conda suite).
- [ ] **Step 5: Golden** (`AWM_CELL_STEP` set and `docs/3D/Dau_han_robot.step` present): teaching the nozzle-mouth
  edge of `LightWELD:1` gives the TCP of `cad_map_head.yaml` within 0.05 mm / 0.1°.
- [ ] **Step 6: Commit** `awm_geometry: export-head TCP teach from a nozzle-mouth edge (CP-61)`.

---

### Task 7: Seam side — part-only edges and stale inputs (`awm_seam`)

**Files:** Modify `awm_seam/joint_seams.py`, `awm_seam/seam_axis.py`. Test `src/awm_seam/test/test_cp60_roles.py`.

**Interfaces:** Consumes `cad_roles.effective_roles`, topology `component_index`. When `jobs/J/roles.json` exists:
an edge whose solid is not `part`, or a joint not between two `part` solids, is refused
(`SeamError('edge E<n> is on <role> component <path>; only part edges can be welded')`). `seam_axis.json` `inputs` add
`model.step`, `roles.json`, `axis_pick.json`, `placement.json`, cell `frames.yaml` when present.

- [ ] **Step 1: Failing tests:** `test_edge_on_rotary_fixture_refused`, `test_joint_part_to_holder_refused`,
  `test_roles_edit_makes_seam_axis_stale` (rewrite roles.json → `jobs.seam_axis_state` style hash check reports stale),
  `test_jobs_without_roles_unchanged` (G1 ring job still builds).
- [ ] **Step 2–4:** `scripts/dx "python3 -m pytest src/awm_seam/test/test_cp60_roles.py -v"` FAIL → implement → PASS;
  `colcon test --packages-select awm_seam awm_path` → 0 fail.
- [ ] **Step 5: Commit** `awm_seam: CP-60 check 5 (part edges only) and roles / axis inputs in seam_axis`.

---

### Task 8: API — cell model (Admin)

**Files:** Modify `awm_api/app.py`, `adapters.py`, `settings.py`; create `awm_api/cell_model.py`; extend
`test/fake_cli.py`; update `docs/generated/contracts/awm_api_sim1.openapi.yaml`. Test `test/test_cell_model_api.py`.

**Endpoints** (all under `/api/v1`; write = Admin, read = Admin + Engineer):

| Method | Path | Does |
|---|---|---|
| POST | `/cell/model` (multipart `step`, `up_axis`) | run: store `cell/cad/cell.step`, `inspect-assembly`, `export-cell-view` |
| GET | `/cell/model` | status: files present, roles, axis pick, last save, stale jobs count |
| GET | `/cell/model/files/{name}` | `cell_components.json`, `cell_view.glb`, `roles.json`, `axis_pick.json` |
| PUT | `/cell/model/roles` | body = assignments; `validate_roles(…, 'cell')` → 422 with the problems; writes roles.json; updates `cell/roles_template.json` |
| POST | `/cell/model/component_view` `{component_path}` | run: `export-view --component` → `cell/cad/views/<key>/` (for the axis pick) |
| PUT | `/cell/model/axis` `{component_path, edge_id, reverse}` | `pick-circle` → `cell/cad/axis_pick.json` |
| POST | `/cell/model/save` | run: `apply-roles` + `export-scene` (no `--job`) |
| POST | `/cell/head/view` · PUT `/cell/head/tcp` `{edge_id, stickout_mm, reverse}` | head view of the tool component; `export-head --tcp-edge` |

- [ ] **Step 1: Failing tests** (fake CLI): `test_cell_import_admin_only` (Engineer 403, Operator 403),
  `test_roles_put_returns_problems_422`, `test_axis_put_writes_pick`, `test_save_runs_apply_roles_then_export_scene`
  (argv order), `test_tcp_put_passes_stickout_and_user`, `test_engineer_can_read_cell_files`.
- [ ] **Step 2–4:** `scripts/dx "python3 -m pytest src/awm_api/test/test_cell_model_api.py -v"` FAIL → implement → PASS;
  `test_openapi_contract.py` PASS.
- [ ] **Step 5: Commit** `awm_api: cell model endpoints (CP-60 S10, CP-61 TCP teach)`.

---

### Task 9: API — job roles, axis, check, gates (Engineer)

**Files:** Modify `app.py`, `adapters.py`, `jobs.py`; OpenAPI. Test `test/test_job_roles_api.py`.

**Endpoints:** `POST /jobs` gains form `assembly: bool` (true → `inspect-assembly` + `export-view --components`;
default false keeps today's import). `GET/PUT /jobs/{id}/roles` (validate `'job'`; GET pre-fills from
`cell/roles_template.json` when no roles.json), `PUT /jobs/{id}/axis` `{edge_id, reverse}`,
`POST /jobs/{id}/check` (run `job-placement`), `GET /jobs/{id}/check`. Gates: `PUT /jobs/{id}/seams` and
`POST /jobs/{id}/plan` → 409 `cad-check-red` when the job has roles.json and cad_check.json is missing, red or older
than its inputs; `PUT /placement` (manual) → 409 for jobs with `source: job_cad`.

- [ ] **Step 1: Failing tests:** `test_roles_prefilled_from_template`, `test_roles_engineer_only`,
  `test_plan_blocked_when_check_red`, `test_plan_blocked_when_check_stale`, `test_yellow_check_allows_plan`,
  `test_job_without_roles_keeps_g16_flow`.
- [ ] **Step 2–4:** FAIL → implement → PASS (+ full `awm_api` package tests).
- [ ] **Step 5: Commit** `awm_api: job roles / axis / cross-check endpoints and gates (CP-60)`.

---

### Task 10: Web — S10 Cell model (Admin)

**Files:** Create `awm_web/src/pages/CellPage.tsx`, `components/RoleTree.tsx`, `components/CirclePick.tsx`,
`lib/roles.ts` (port of `effective_roles` + role colours); modify `lib/route.ts` (`#/cell`), `lib/api.ts`, `App.tsx`.
Test: vitest `roles.test.ts`, `route.test.ts`; Playwright smoke `e2e/cell.spec.ts`.

UI: upload + up axis; tree with role dropdown per node (inherited roles shown greyed, override possible), exclude
toggle, problems list from the 422; 3D `cell_view.glb` coloured by role (robot grey, tool orange, rotary_fixture blue,
part green, static light grey, excluded hidden); robot base shown as a frame at the `robot` node placement for
confirmation (spec 2.1.4); tab "Positioner axis": select component → component view → click a
circular edge → arrow + radius, "Reverse", Save; tab "Tool / TCP": click nozzle-mouth edge → arrow out of the nozzle,
"Reverse", stick-out mm (default 0), Save; "Save cell model" runs the save and shows the stale-jobs count.

- [ ] **Step 1: Failing tests:** `roles.test.ts` (same cases as Task 1 `effective_roles`), `route.test.ts` (`#/cell`),
  e2e: Operator sees no S10 link; Admin assigns a role and saves (fake API).
- [ ] **Step 2–4:** `scripts/dx "cd src/awm_web && npm test && npm run typecheck"` FAIL → implement → PASS.
- [ ] **Step 5: Commit** `awm_web: S10 cell model — roles, positioner-axis pick, Tool / TCP teach`.

---

### Task 11: Web — S2 tree, roles, axis pick, check banner (Engineer)

**Files:** Modify `pages/JobPage.tsx`, `pages/JobList.tsx` (import option "assembly (positioner + fixture + part)"),
reuse `RoleTree`, `CirclePick`; `lib/api.ts`. Test vitest + e2e `e2e/job_roles.spec.ts`.

UI: tree panel next to the 3D view (all solids, role colours); edges of non-`part` solids not pickable; axis pick
on rotary_fixture edges; "Run check" → banner green / yellow / red with each check's value vs tolerance and φ;
red disables seam picking and "Plan all seams"; stale check shows "re-run check".

- [ ] **Step 1: Failing tests:** `test pick ignored on non-part solid`, `banner red disables plan`,
  `banner shows phi and values`.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `awm_web: S2 roles tree, positioner-axis pick and cross-check banner (CP-60)`.

---

### Task 12: Golden end-to-end and demo (lead, after merges)

- [ ] Full suite: `scripts/dx "colcon build --symlink-install && colcon test && colcon test-result --verbose"` →
  0 errors / 0 failures (skips only the documented ones); conda suite with `AWM_CELL_STEP` → 0 skipped.
- [ ] Browser chain on a demo repo copy: S10 import cell CAD → roles (from the old map via template) → axis pick →
  TCP teach on the nozzle mouth → save; S1 import the extracted job STEP (assembly) → S2 roles pre-filled → axis pick →
  check green → pick J1, J2 → plan at 2.3 mm/s → J1 and J2 100 % sync (as G1.3) → S4 program, play.
- [ ] Update `.worktrees/demo/RUN_DEMO_G1.md` (new steps), `PROGRESS.md`; list wiki updates for Truong.
