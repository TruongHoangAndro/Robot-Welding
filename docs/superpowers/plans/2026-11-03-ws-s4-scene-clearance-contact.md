# WP-S S.4 — collision scene (H-BOX first), clearance, contact and entry gate G2 — Implementation Plan

**Decision of 2026-11-03 (Truong):** H-BOX head model first, agents in sequence (at most 2 at a time). H-CAD hulls
only after S.5 runs with H-BOX. S.3 (`ws-s3-state-space`, review PASS) is not merged yet: Part C stacks on it.

**Goal:** a MoveIt planning scene of the cell for the spike — part with per-triangle `face_id`, jig / holders / positioner
rotating set, Pismo frame, UR5e at its CAD pose, the H-BOX head with `nozzle_tip` / `nozzle_body` and `tcp_laser`, a
7th joint `positioner_joint` — plus the S.4 checks: validity with 3 mm padding (F4), unpadded clearance (F5), nozzle
contact (F9), and entry gate G2 (C1 scene sanity).

**Spec:** PLAN.md WP-S S.4 (and the parts of T0.2 / T0.3 it needs); TST-002 Rev 0.3 DRAFT §3.1 (H-BOX), §3.2, §5.1
F4, F5, F9, F10, §5.2 G2; DES-001 Rev 0.6 DRAFT §5.3 (frames), §5.4 (collision model; no ACM head ↔ part, B-04);
CP-20 (positioner as modelled), CP-23 (Fusion assemblies, `REF_*` markers not yet present → component placements
are an **Assumption**, recorded per result, OI-23); D19 v2 schema `src/awm_core/awm_core/schemas/v2/placement.schema.json`.

**Data rule:** component names, poses and the role map are **data**, never source. The spike role map lives in
`docs/generated/spike/cad_map_cell.yaml` (record, TST-002 §9) and is passed by path. Generated meshes / manifests go
to a git-ignored spike repository `<ws>/.worktrees/spike_repo/` (cell part `cell/`, job `jobs/564082_A/`).

---

## Part A — `cad-geometry` (worktree `.worktrees/cad-geometry`, branch `ws-s4a-scene-export` from GotoCode)

A1. `$AWM_OCC_PYTHON -m awm_geometry.cli inspect-assembly --step <file> --out <json>`: component tree with path,
    name, placement (4×4, mm), bbox (mm), solid count. Test on a synthetic assembly; run once on the cell CAD and
    commit the output summary (paths, names, bboxes) to `docs/generated/spike/cell_components.md`.

A2. Lead + Truong write `docs/generated/spike/cad_map_cell.yaml` from A1 (roles: `world_ref`, `ur_base`,
    `positioner_axis`, `positioner_rotating` (holders, jig plate, shaft side), `cell_static` (Pismo frame, table,
    motor / belts housing), `part` (component `564082-Assembly-3 parts`), `ignore`; file up axis `+Y`; default-deny =
    unmapped components are an error). **Stop after A1 and report; the lead writes the map.**

A3. `export-scene --step <file> --map <cad_map.yaml> --repo <spike_repo> --job <job> --chord-near-mm 0.1`:
    - world frame = CAD frame rotated to Z up (per map); origin at the `ur_base` component placement projected on its
      mounting plane (**Assumption** until `REF_WORLD` / `REF_UR_BASE`). Writes `cell/model/frames.yaml` (world_T_base,
      world_T_positioner_axis, axis direction) in mm / quaternion.
    - `cell_static` and `positioner_rotating` objects: convex hull per component (STL, mm) in `cell/model/` with a
      manifest (object id, role, parent `world` or `positioner_flange`, pose, mesh SHA-256).
    - part: collision mesh (binary STL, mm) with chord ≤ 0.1 mm near joint edges and coarser elsewhere, plus
      `cache/collision_faces.json` (triangle index → face_id, same ids as D20 / edges.json via `part.load_part`);
      `jobs/<job>/placement.json` v2 (`source: fixture_cad`, `verified: false`, parent `positioner_flange`).
    - tests: synthetic assembly → manifest validates against `v2/placement`; hull contains all vertices; face_id map
      covers every triangle; ids equal D20 ids; real run on the cell CAD (skip only `AWM_CELL_STEP not set`).

## Part B — `cell-sim` (worktree `.worktrees/cell-sim`, branch `ws-s4b-cell-model` from GotoCode; in parallel with A)

B1. `awm_description` cell v2 xacro (keep the old MIG model loadable until T0.3 removes it): UR5e from `ur_description`;
    pose of `base_link` from xacro args (default identity; real values from A3 `frames.yaml` via a launch argument);
    `positioner_joint` (revolute, axis and origin from args, limits ±400° Assumption), links `positioner_axis`,
    `positioner_flange`; static meshes and rotating meshes loaded from a manifest path argument (no paths in source).
B2. H-BOX tool on `tool0` (TST-002 §3.1, every value an xacro arg with the documented default, marked Assumption):
    mount box, head body box 194 × 35 × 381 mm, nozzle cylinder Ø20 × 40 mm split into `nozzle_body` and `nozzle_tip`
    (last 5 mm), conduit capsule Ø30 × 300 mm at the head rear; beam axis = `tool0` Z; `tcp_laser` on the beam axis at
    the nozzle end; the `nozzle_tip` collision geometry stops s_c = 0.3 mm before `tcp_laser` (F9).
B3. SRDF: group with the 6 robot joints (+ group with `positioner_joint`); disable only adjacent-link pairs and pairs
    that can never collide by kinematics; **no** entry between any head link and the part or holders (B-04); padding
    3 mm on every link except `nozzle_tip` (F4).
B4. ros2_control with `mock_components` for 7 joints, one controller manager; `sim_mock` launch (T0.4 subset) starts
    `move_group` with this model. Tests: xacro → URDF parses; FK of `tcp_laser` = tool0 + documented offset; launch
    smoke test shows 7 joints in `/joint_states`; `check_state_validity` home = valid.

## Part C — `planner` (worktree `.worktrees/planner`, new branch `ws-s4c-checks` from `ws-s3-state-space`, after A and B)

C1. `awm_path.backends.move_group`: load the scene from the D19 manifest + cell manifest (attached to
    `positioner_flange` / `world`); set the positioner angle; `check(states, theta)`; record the scene hash.
C2. Clearance (F5): method = validity check with the head / mount / conduit / nozzle_body padding set to exactly the
    clearance (5 mm weld, 10 mm air) on unpadded geometry, via a planning-scene diff; method recorded per result.
C3. Contact (F9), pure Python in `awm_path.core.contact` with numpy only: sample the `nozzle_tip` end disc and rim at
    the candidate pose; distance to the part triangles; pass = the contact point(s) lie on the seam faces (D04
    `faces.n1` / `faces.n2`) within tolerance and no other face is within s_c. Uses `collision_faces.json`.
C4. Entry gate G2 (C1): 10 scripted colliding poses invalid, 10 free poses valid, plus the three contact-zone poses
    of TST-002 §5.2 (tip 0.5 mm into a non-seam face; rim into a corner wall; flat tip tilted 8° on a flat face).
    Poses come from a data file in the spike records, not from source. Report the log; G2 fail stops S.5.
C5. Fill the S.3 `NodeScore` from C1–C3 and write the scene columns of `candidates.csv`; measure the time per state
    (C5 budget).

Every part ends with the AGENT_TEAM.md §3 report; nothing is merged without Truong's choice.
