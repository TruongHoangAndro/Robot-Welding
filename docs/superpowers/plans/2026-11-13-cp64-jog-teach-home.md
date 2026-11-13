# CP-64 Manual jog page and Teach home — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A new Admin web page (S11) that jogs the UR5e (joint + Cartesian) and the positioner (joint) in the browser, moves to a typed pose, and teaches the cell home (D22) — model + analytic IK only, no controller.

**Architecture:** Browser (`JogPage.tsx`) holds the saved cell URDF, does FK/limit checks locally (`lib/robot.ts`), and builds Cartesian step targets; it calls `awm_api` for analytic IK and for reading/writing the home file. `awm_api` computes kinematics in-process with `awm_path.core.kinematics.UrAnalyticKinematics` (mm/deg, no MoveIt, no ROS graph) and composes the `tool0 ↔ tcp_laser` offset from the taught cell TCP (`head.yaml`). Home is joint angles in `cell/model/home.json`.

**Tech Stack:** Python 3.12 / FastAPI (`awm_api`), `awm_path.core.kinematics` (wraps `ur_analytic_ik`), JSON Schema (`awm_core`), React + TypeScript + three.js / urdf-loader (`awm_web`), pytest, vitest.

**Spec:** `docs/superpowers/specs/2026-11-13-jog-teach-home-design.md`

## Global Constraints

- **Simulation only.** No ROS control graph, no `controller_manager`, no mock/URSim/real hardware is opened or commanded by any task here. Kinematics are numeric only.
- **Units mm / deg / s in all project data and API** (ADR-14). Metres/radians never leave the kinematics boundary; `UrAnalyticKinematics` already works in **mm / deg**. No ad-hoc `*1000` / `/1000`.
- **Poses** are represented as `{ "xyz_mm": [x,y,z], "quat_xyzw": [x,y,z,w] }` (same shape as `head.yaml` `tool0_T_tcp_mm`).
- **Role:** S11 and its write endpoints are **Admin** (`CELL_WRITE = ('Admin',)`); reads are `CELL_READ`. Role checks are hierarchical (CP-63: an Admin already satisfies any Engineer gate; no Engineer gate is added here).
- **Run ROS/pytest inside the container:** `scripts/dx "python3 -m pytest <path> -v"`. Web tests: `scripts/dx "cd src/awm_web && npm run test"` and typecheck `npm run typecheck`.
- **No absolute paths in source**; use `Settings` (`head_dir`, `frames_yaml`, `repo_root`).
- **Do not** wire D22 into the planner seed or runtime "go home" (follow-up, Truong 2026-11-13).

## Review Focus

- **Radian/metre leak:** an endpoint or the UI emitting radians or metres instead of deg/mm → pinned by Task 2/3 round-trip tests asserting deg/mm values and by the browser jog-math unit test.
- **Missing cell TCP:** `head.yaml` absent or without `tool0_T_tcp_mm` → `/cell/fk`/`/cell/ik` for `tcp_laser` must return a 409 problem (not 500), and the UI must disable Cartesian tools while keeping joint jog + teach home → Task 2 test `test_ik_no_tcp_409`, Task 5 UI guard.
- **Unreachable / joint-limit / wrist-singular target:** `/cell/ik` must return a typed problem, never a 500 or a silent bad solution → Task 2 tests `test_ik_out_of_reach`, `test_ik_reports_singular`.
- **Nearest-branch flip:** `/cell/ik` returning a far IK branch so the model jumps → Task 2 test `test_ik_picks_nearest_to_current`.
- **Stale home after cell re-import:** `home.json` kept after the cell STEP changed → Task 3 test `test_home_stale_when_frames_change` (staleness by `frames.yaml` hash).

---

## Task 1: D22 home schema (awm_core)

**Files:**
- Create: `src/awm_core/awm_core/schemas/home.schema.json`
- Create: `src/awm_core/awm_core/schemas/examples/home.json`
- Test: `src/awm_core/test/test_home_schema.py`

**Interfaces:**
- Produces: schema id `home` loadable by the existing `awm_core` schema loader (same pattern as `calibration.schema.json`); document shape `{schema_version, created_by, created_at, source:"taught", inputs:[{name,sha256}], robot_joints_deg:{<6 UR joint names>:number}, positioner_joint_deg:number}`.

- [ ] **Step 1: Write the failing test** — in `test_home_schema.py`: `test_valid_home_accepted` (the example validates), `test_missing_positioner_rejected`, `test_extra_joint_rejected` (robot_joints_deg requires exactly the 6 UR joint names: `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`), `test_source_must_be_taught`.
- [ ] **Step 2: Run** `scripts/dx "python3 -m pytest src/awm_core/test/test_home_schema.py -v"` → FAIL (schema missing).
- [ ] **Step 3: Write `home.schema.json`** — `required` header fields reuse `common.schema.json` `$defs` as the other schemas do; `robot_joints_deg` an object with the 6 named number properties and `additionalProperties:false`; `positioner_joint_deg` a number; `source` const `"taught"`. Write `examples/home.json` with a plausible home (deg).
- [ ] **Step 4: Run** the test → PASS.
- [ ] **Step 5: Commit** `awm_core: D22 home.schema.json (CP-64)`.

## Task 2: Kinematics endpoints `/cell/fk`, `/cell/ik` (awm_api)

**Files:**
- Create: `src/awm_api/awm_api/jog.py` (kinematics helpers + pose <-> matrix in mm/deg)
- Modify: `src/awm_api/awm_api/app.py` (register two routes near the other `/cell/*` routes; constants if needed)
- Test: `src/awm_api/test/test_jog_api.py`

**Interfaces:**
- Consumes: `awm_path.core.kinematics.UrAnalyticKinematics` (`fk(q_deg)->4x4 base_link_T_tool0`, `ik(base_link_T_tool0)->List[q_deg]`); `awm_api.cad_api.head_tcp_record` / `head.yaml` `tool0_T_tcp_mm` for the `tool0->tcp_laser` offset; `Settings.head_dir`.
- Produces:
  - `pose_to_matrix(pose: dict) -> np.ndarray` and `matrix_to_pose(T) -> dict` (mm, quat_xyzw).
  - `tool0_T_tcp(settings) -> Optional[np.ndarray]` (None if no taught TCP).
  - `POST /api/v1/cell/fk` body `{joints_deg:[6]}` → `{tool0:pose, tcp_laser:pose|null}` (CELL_READ).
  - `POST /api/v1/cell/ik` body `{target_tcp_laser:pose, current_joints_deg:[6]}` → `{joints_deg:[6]}` or a `application/problem+json` (CELL_WRITE); problem types `/problems/ik-no-tcp` (409), `/problems/ik-unreachable` (422), `/problems/ik-singular` (422).

- [ ] **Step 1: Write the failing tests** in `test_jog_api.py`:
  - `test_fk_round_trips_ik`: `fk(q)` then `ik` of that pose returns a solution within 1e-3 deg of `q` (uses the module helpers directly).
  - `test_fk_endpoint_returns_both_frames`: POST `/cell/fk` with a config → `tool0` and `tcp_laser` poses present, mm values finite; with no `head.yaml` → `tcp_laser` is null.
  - `test_ik_picks_nearest_to_current`: two valid branches exist; the returned `joints_deg` is the one closest to `current_joints_deg`.
  - `test_ik_out_of_reach`: a far target → 422 `/problems/ik-unreachable`.
  - `test_ik_reports_singular`: a wrist-singular target → 422 `/problems/ik-singular`.
  - `test_ik_no_tcp_409`: no `head.yaml` → 409 `/problems/ik-no-tcp`.
  - `test_ik_requires_admin`: `op` and `eng` → 403; `adm` → not 403 (CP-63).
- [ ] **Step 2: Run** `scripts/dx "python3 -m pytest src/awm_api/test/test_jog_api.py -v"` → FAIL.
- [ ] **Step 3: Implement `jog.py` and the two routes.** IK path: compose `base_link_T_tool0 = target_tcp · tool0_T_tcp⁻¹`; call `UrAnalyticKinematics().ik`; drop wrist-singular solutions using the planner's margin (`awm_path.core` wrist-2 margin / `min_wrist_singularity_margin_rad`); if no solution → unreachable; pick the solution minimising `sum|q − current|`; return deg. Reuse the existing `require(*CELL_WRITE)` / `require(*CELL_READ)` and `ProblemError`.
- [ ] **Step 4: Run** the tests → PASS.
- [ ] **Step 5: Commit** `awm_api: /cell/fk and /cell/ik analytic kinematics (CP-64)`.

## Task 3: Home read/write `/cell/home` (awm_api)

**Files:**
- Modify: `src/awm_api/awm_api/app.py` (two routes) and `src/awm_api/awm_api/jog.py` (home read/write helper) — or a small `home.py`
- Test: `src/awm_api/test/test_home_api.py`

**Interfaces:**
- Consumes: `Settings.head_dir`→ sibling `cell/model/home.json` path; `Settings.frames_yaml` (hash input for staleness); `awm_core` `home` schema (Task 1); `jsonio.make_header` / `jsonio.write_json(path, doc, schema)` and `sha256_file`.
- Produces: `GET /api/v1/cell/home` → `{home, state}` where `state ∈ {ok, stale}` (404 `/problems/no-home` if absent) (CELL_READ); `PUT /api/v1/cell/home` body `{robot_joints_deg:{...}, positioner_joint_deg:number}` → the written record (CELL_WRITE).

- [ ] **Step 1: Write the failing tests** in `test_home_api.py`:
  - `test_put_then_get_home`: PUT a config as `adm` → 200/201; GET returns it with `state:"ok"`, `created_by:"adm"`, `source:"taught"`.
  - `test_get_home_404_when_absent`.
  - `test_put_home_validates_joints`: wrong joint set → 422.
  - `test_home_stale_when_frames_change`: after writing, change `frames.yaml` → GET `state:"stale"`.
  - `test_home_requires_admin`: `op`/`eng` PUT → 403; `adm` → not 403.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement** the helper + routes; write through the `home` schema; `inputs` = `[frames.yaml sha256]`; staleness compares stored vs current hash (same pattern as `jobs.document_state`).
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `awm_api: /cell/home read/write D22 (CP-64)`.

## Task 4: Browser jog math (awm_web lib)

**Files:**
- Create: `src/awm_web/src/lib/jog.ts`
- Test: `src/awm_web/tests/jog.test.ts`

**Interfaces:**
- Consumes: three.js `Matrix4`/`Vector3`/`Quaternion`; `lib/units.ts` (`poseMmToMatrix`, mm scale).
- Produces: `stepTarget(currentTcp: Matrix4, axis: 'x'|'y'|'z', kind:'lin'|'rot', amount: number, frame:'tool'|'base'): Matrix4` and `matrixToPoseMm(m): PoseMm` → the `tcp_laser` target sent to `/cell/ik`.

- [ ] **Step 1: Write the failing tests** in `jog.test.ts`:
  - `stepTarget tool-frame +X translates along the tool axis` (a rotated TCP moves along its own X, not world X).
  - `stepTarget base-frame +Z translates along world Z regardless of orientation`.
  - `stepTarget rotation about tool Z changes orientation only, not position`.
  - `matrixToPoseMm returns mm and unit quaternion`.
- [ ] **Step 2: Run** `scripts/dx "cd src/awm_web && npm run test -- jog"` → FAIL.
- [ ] **Step 3: Implement** `jog.ts` (compose `currentTcp` with a step transform pre- (base) or post- (tool) multiplied).
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `awm_web: jog step math (CP-64)`.

## Task 5: S11 Jog page + route + nav + API client

**Files:**
- Create: `src/awm_web/src/pages/JogPage.tsx`
- Modify: `src/awm_web/src/lib/route.ts` (add `jog` route), `src/awm_web/src/Root.tsx` (dispatch + Admin nav button), `src/awm_web/src/lib/api.ts` (`cellFk`, `cellIk`, `getCellHome`, `putCellHome`)
- Test: `src/awm_web/tests/jogpage.test.ts` (logic/guards only; reuse the project's existing component-test style)

**Interfaces:**
- Consumes: Task 2/3 endpoints via `api.ts`; `lib/robot.ts` (`parseRobot`, `setJoints`, `limitViolations`, `rootT`, `TCP_LINK`, `FLANGE_LINK`); Task 4 `jog.ts`; the cell scene loader used by `CellPage`/`SimPage`.
- Produces: `#/jog` route, Admin-only nav entry, page with joint jog (6 UR + positioner), Cartesian jog (Tool/Base toggle), move-to, Teach home, and a readout showing **both** `tcp_laser` and `tool0` (browser FK via `rootT`).

- [ ] **Step 1: Write the failing tests** in `jogpage.test.ts`:
  - `canEditJog is Admin only` (a small guard like `access.ts`, or reuse `role==='Admin'`): `Admin` true, `Engineer`/`Operator` false.
  - `route '#/jog' parses to {page:'jog'}`.
  - `readout shows both tool0 and tcp_laser labels` / `Cartesian controls disabled when no cell TCP` (logic-level, mirroring `roles.test.ts`/`access.test.ts` style).
- [ ] **Step 2: Run** `scripts/dx "cd src/awm_web && npm run test -- jogpage"` → FAIL.
- [ ] **Step 3: Implement** `route.ts` case, `Root.tsx` dispatch + `{session.role === 'Admin' && <a href="#/jog">S11 jog</a>}`, `api.ts` clients, and `JogPage.tsx`. Joint limits via `limitViolations` (covers robot + positioner from the URDF). Cartesian jog: `jog.ts` step → `/cell/ik` → `setJoints`; on a problem, show it and do not move. Teach home: PUT current joints.
- [ ] **Step 4: Run** `scripts/dx "cd src/awm_web && npm run typecheck && npm run test"` → PASS; then `npm run build`.
- [ ] **Step 5: Commit** `awm_web: S11 jog page + teach home (CP-64)`.

## Task 6: OpenAPI contract + DES-001 Rev 0.5 doc update

**Files:**
- Modify: `docs/generated/contracts/awm_api_sim1.openapi.yaml` (the new `/cell/fk`, `/cell/ik`, `/cell/home`)
- Modify: `docs/generated/release_rev0.5/APLSMN601-DES-001_Software_Architecture_Rev0.5.md` (§4.5 S11, §5.1/§5.2 D22, §7.3, §7.4) — logged under CP-64 (already in the change proposal)
- Test: the existing API-contract test, if one asserts the OpenAPI matches the routes

- [ ] **Step 1:** add the three paths + schemas to the OpenAPI, with role notes (Admin for writes).
- [ ] **Step 2:** run the contract/`awm_api` test suite `scripts/dx "python3 -m pytest src/awm_api/test -q"` → PASS (0 unexpected skips).
- [ ] **Step 3:** add S11 to §4.5 screen table, D22 to §5.1/§5.2, endpoints to §7.4, S11 Admin to §7.3; one OI line for real-robot-jog safety and one for D22→planner/"go home".
- [ ] **Step 4:** `docs:` commit `CP-64 S11 jog + D22 in DES-001 Rev 0.5 and OpenAPI`.

## Final verification

- [ ] `scripts/dx "colcon build --symlink-install && colcon test && colcon test-result --verbose"` (AWM_OCC_PYTHON set; only allowed skip = `AWM_CELL_STEP not set`).
- [ ] `scripts/dx "cd src/awm_web && npm run typecheck && npm run test && npm run build"`.
- [ ] Manual: restart the demo web server, log in as `adm`, open S11, joint-jog, Cartesian-jog, teach home, confirm `cell/model/home.json` written and GET returns it.
