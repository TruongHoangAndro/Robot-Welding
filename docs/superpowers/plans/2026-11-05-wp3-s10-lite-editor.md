# WP3 S10-lite — cell model role editor page — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A browser page that shows the cell CAD component tree in 3D, lets the person assign a role and display name
to every component, validates the map and downloads `cad_map.yaml`.

**Architecture:** Same app and data path as S2-lite (`awm_web`, files served from `AWM_VIEW_DIR`). Pure validation /
serialisation logic in `src/lib/cadMap.ts` (unit-tested with vitest), UI in a second page of the app. No server write
path (contract: download only; saving through `awm_api` comes with T3.1).

**Tech Stack:** React 19 + TypeScript, react-three-fiber / drei, three 0.180, vitest 5; YAML written by a small
serializer (no new dependency unless the contract needs one — ask the lead).

**Spec:** `docs/generated/contracts/S10_lite_cell_model.md` (v0.1, frozen 2026-11-03), CP-54,
schema `src/awm_core/awm_core/schemas/v2/cad_map.schema.json` + example; reference map
`docs/generated/spike/cad_map_cell.yaml`; data `.worktrees/spike_repo/cell_components.json`, `cell_view/cell_view.glb`.

## Global Constraints

- Roles fixed: `part`, `positioner_rotating`, `cell_static`, `head_cad`, `ignore`; frame picks `ur_base`,
  `positioner_axis`. New roles need a contract change.
- Rules 1–5 of the contract enforced before download (default deny; one role per component; exactly one `ur_base`,
  `positioner_axis` present; `part` names the job part(s), `head_cad` entries name a depth; display names only for
  existing instance names, never empty).
- Names matched on `instance_name`, never by splitting paths on `/`.
- Display only: glb is never used for geometry; lengths in mm, file frame, up axis from `asset.extras.up_axis`.
- Page shows whether a role belongs to the cell (Admin) or the job fixture (Engineer); no login.
- No absolute paths; TypeScript strict; existing S2-lite viewer unchanged.

## Review Focus

1. Ancestor role + different descendant role → error, except `ignore` on an ancestor with nothing mapped below.
2. Duplicate instance names in different branches (e.g. the nested `ASSY, Welding tool:1`) → must be addressable
   (contract uses `path` + `depth` entries for `head_cad`).
3. Loading an existing `cad_map.yaml` whose names no longer exist → listed, not dropped silently.
4. Large glb (23 MB): loading progress and no freeze on selection (index lookup, not scene traversal per click).
5. Round trip: load `docs/generated/spike/cad_map_cell.yaml` → download → semantically equal and valid against the
   v2 schema (Python test in `awm_core` validates the fixture output).

---

### Task 1: `cadMap.ts` — coverage, validation, YAML

**Files:** Create `src/awm_web/src/lib/cadMap.ts`, `src/awm_web/src/lib/cadMap.test.ts`, fixture
`src/awm_web/test/fixtures/cad_map_cell.yaml` (copy of the spike map) + trimmed `cell_components` fixture.

**Interfaces:** `parseCadMap(text) -> CadMap`, `resolveCoverage(tree, map) -> Map<index, {role, via}>`,
`validateCadMap(tree, map) -> Problem[]` (`{rule: 1..5, component?, message}`), `serializeCadMap(map) -> string`.

- [ ] Step 1: failing tests — spike map on spike tree: 0 problems; remove `Jig:1` → rule 1 problem naming it; add
  `Jig:1` to `ignore` too → rule 2; two `ur_base` → rule 3; empty display name → rule 5; serialize → parse round trip.
- [ ] Step 2: implement; Step 3: `npm test` green; Step 4: Python test in `src/awm_core/test/` validating the
  serialized fixture output against `v2/cad_map`; Step 5: commit `awm_web: S10-lite cad_map logic`.

### Task 2: S10-lite page

**Files:** Create `src/awm_web/src/components/CellEditor.tsx` (+ small tree / role panel components); modify
`App.tsx` (page switch `?page=cell`), `src/awm_web/README.md`.

- [ ] Step 1: failing component test (vitest + jsdom if already available, else logic-level test of the page state
  reducer): assigning a role updates coverage; download disabled while problems exist.
- [ ] Step 2: implement: tree with role chip and Admin / Engineer marker, 3D `cell_view.glb` with node highlight by
  `extras.index`, role dropdown, display name field, positioner axis from a picked component's `main_cylinder`,
  problems list, download button.
- [ ] Step 3: `npm run build` and `npm test`; manual check on the spike data (screenshot for Truong).
- [ ] Step 4: commit `awm_web: S10-lite cell role editor page`.
