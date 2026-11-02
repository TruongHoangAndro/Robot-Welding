# APLSMN601 — implementation plan Rev 0.6 (spike → Sim-1 → Sim-2 → hardware gates)

Status: **draft for review by Truong Hoang**, written by the software agent on 2026-11-02. It replaces the plan
Rev 0.4 draft of 2026-10-27 (in git: `git show HEAD:PLAN.md`). No code has been changed for this plan yet.

Sources:
- DES-001 Rev 0.6 DRAFT (`docs/generated/drafts/`), decisions B-01…B-25 of the Architecture Review Board of
  2026-10-30 (`docs/generated/arch_review_board_2026-10-30.md`; AI reviewer agents, input for the human review);
- TST-002 Rev 0.3 DRAFT (`docs/generated/drafts/`, frozen spike inputs);
- DES-003 Rev 0.5 (function register), change proposal CP-9…CP-51.

Items that depend on an open escalation (E-01…E-08, DES-001 OI-36…OI-44) are marked **[E-xx]**. They are planned
with the board recommendation as default and change if the decider chooses otherwise.

## 1. Stages and dates (DES-001 Rev 0.6 §2.3)

| Stage | Date | Exit (scripted acceptance run) |
|---|---|---|
| Contract freeze | Wed 2026-11-04 | Schemas v2 + example files, Sim-1 OpenAPI, minimal D20 exporter, `awm_interfaces` v1 draft; one-hour review by Truong (B-13) |
| TST-002 decision | Fri 2026-11-06 **[E-01]** | Spike results on frozen inputs (TST-002 Rev 0.3): Go / Pivot / No-Go / not decided (+≤ 3 working days) |
| Sim-1 | Fri 2026-11-20 | J1 Mode C end to end in the browser; J2 robot-only = 100 % path or rejection with failing points; golden-job planner regression and FK consistency test green |
| Sim-2 | Fri 2026-12-04 | Simulation release replayed by M8 / M9 / M10-sim on URSim + mock positioner; fault-injection subset; commanded vs actual on URSim |
| HW-1 | after Sim-2, date in PM-004 **[E-04]** | Calibration and dry run, emission blocked by hardware; entry criteria in DES-001 §2.3 |
| HW-2 | after HW-1 **[E-04]** | Trial weld; entry criteria in DES-001 §2.3 |

Phase 1 deliverable on 2026-12-04 = the Sim-2 exit (board recommendation, **[E-04]**).

Golden part: `564082-Assembly-3 parts` (pipe nipple + solid + elbow, **three separate solids**). Two closed butt
joints, Ø 26.67 mm: **J1** nipple ↔ solid, coaxial with the positioner axis → Mode C; **J2** solid ↔ elbow, 76.6 mm off
the axis, axis perpendicular to it → robot-only first, Mode B only if robot-only < 100 % (OI-27).

## 2. Code baseline: keep / replace (B-08, closes DES-001 OI-34)

| Area | Delivered 2026-10-19 (Rev 0.2) | Decision | Action |
|---|---|---|---|
| `awm_core` | headers, hashes, atomic writes, schemas v1 (m) | **Keep**, migrate | mm adapter, schemas v2, context record (B-10), `cell.yaml` + D08 by component |
| `awm_geometry` import / heal / catalogue / sampling | per-solid face normals | **Keep**, fix | Joint-aware weldment shape (B-02); D20 exporter |
| `awm_seam` | edge ids, attributes, re-match | **Keep**, migrate | n1 / n2 with source-solid id; joint-edge resolution |
| `awm_path` core | KDL Cartesian + pick_ik seeds, one service call per state | **Replace** | Spike code becomes the M4 library (B-06); written in mm |
| `awm_process` | MIG, weave | **Replace** | LightWELD WPS v2 with ψ window fields (B-05) |
| `awm_description` | MIG torch, hand-written boxes | **Replace** | UR5e + head / mount + positioner + A18 scripted cell; no ACM head ↔ part (B-04) |
| `awm_program` builder + FK check | 6 joints, MIG events | **Keep**, extend | 7 joints; D09 = exact commanded trajectory in Sim-2 (B-15) |
| `awm_verify` checks / replay / release lock | job-level `release.json` overwritten | **Keep**, change | Frozen per-revision bundle, active pointer per scope (B-09); scene rebuilt from the bundle |
| `awm_interfaces` | MIG-typed WeldEvent | **Replace** | v1 typed contract (B-17) |
| Test jobs (plate, pipe) | generated in m | Regenerate | Not migrated; plus one 2-solid test part |

M6 / M7 stop importing `awm_path` internals; runtime packages depend only on `awm_core`, `awm_interfaces` and the
read-only part of `awm_motion` (B-06, CI test).

## 3. Work packages

Each task lists its test. "Sim" = needs the ROS sim launch; everything else runs with plain pytest.

### WP-S — Spike = M4 core (2–5 Nov, TST-002 Rev 0.3)

| # | Task | Test |
|---|---|---|
| S.1 | Joint-aware weldment shape: general fuse of the part solids (fuzzy 0.05 mm, Assumption), faces keep the source-solid id; n1 / n2 from the exterior faces of the two solids. Fallback by 3 Nov: tube rule n1 = n2 = radial, marked Assumption (B-02) | **Entry gate**: J1 / J2 seam-frame Z radial within 0.5° on the real 3-solid assembly |
| S.2 | M4 library (pure Python, no rclpy, mm) behind `KinematicsBackend` (ur-analytic-ik, version pinned) and `SceneBackend` (move_group services first) (B-06) | IK vs FK < 0.01 mm on random states |
| S.3 | State space (B-03): node = (point, ψ, configuration label, unwrapped joints); 2π offsets at the first layer; pitch Δs ≤ min(3 mm, Δq_max / frame rotation rate); singularity metrics; deterministic tie-break | Pure pytest on synthetic ladders |
| S.4 | Clearance and contact (B-03, B-04): validity with 3 mm padding; C2 / C4 on unpadded geometry; nozzle geometry stops s_c = 0.3 mm short of the contact point; contact check by face_id; no ACM head ↔ part | **Entry gate**: C1 scene sanity incl. contact-zone poses |
| S.5 | Sweep + ladder DP on J2 robot-only, two ψ cases (free; ±15° wire window) × two head models (CAD hulls; TST-002 §3.1 box) (B-05) | C2, C3, C5, C8 table |
| S.6 | J1 Mode C check: one robot pose, head vs rotating part + holders over 360° + overlap with the B-03 angular step | Clearance report (does not gate) |
| S.7 | Records: commit, input hashes, backend, ψ case and head model per result; TST-002 Rev 0.3 §11 filled | Decision record on 6 Nov |

### WP0 — Foundations (2–13 Nov, in parallel with the spike where possible)

| # | Task | Test |
|---|---|---|
| T0.1 | mm migration of **kept code only** (B-08): `awm_core.units` as the only converter; schemas v2; readers refuse v1; static scan for scale literals. `cell/config/cell.yaml` (Admin-owned, hashed: limits, clearances, padding, acceptance and path parameters, positioner range, PLC timeouts) + unhashed software defaults; D08 v2 by component; context record + compare function + policy table (B-10) | Adapter round trip; schema tests; source scan; context compare tests |
| T0.2 | Scripted A18 cell import with `cad_map.yaml`, default-deny mapping, frames from component placements (Assumption until `REF_*`, OI-23); D14 / D19 one owner per scene object; scene manifest shared by planner and browser (B-07) | Real-file test (slow mark) + synthetic assembly; FK consistency test |
| T0.3 | Cell model v2: UR5e from `ur_description`, head + mount, `nozzle_tip` / `nozzle_body`, `tcp_laser`, `positioner_joint`; part, fixture and clamps attached to `positioner_flange` | xacro → URDF parse; FK of `tcp_laser` vs CAD |
| T0.4 | ros2_control: one controller manager, robot + positioner trajectory controllers, `mock_components`. Align launch names (OI-33 closed with the board) | Launch smoke test: 7 joints in `/joint_states` |
| T0.5 | Contract freeze, **Wed 4 Nov** (B-13): schemas v2 for D04 (+ seam geometry), D07, D09, D10 (context, bundle layout), D19 (scene manifest), D20; Sim-1 OpenAPI; one example file per schema from a hand-made J1 Mode C case; minimal D20 exporter on the golden part; `awm_interfaces` v1 draft (B-17) | Schema validation of the examples; review by Truong |
| T0.6 | Environment: ur-analytic-ik venv, Node.js, **URSim + ur_robot_driver + External Control URCap** moved here from WP4 (B-19); all in `docker/Dockerfile` | Version report; URSim connects on a private Docker network |
| T0.7 | Company-controlled remote + self-hosted CI runner with the ROS image (B-23) **[E-06]** | CI runs the unit + contract tests |

### WP1 — Planner complete for Sim-1 (9–20 Nov, after a Go)

| # | Task | Test |
|---|---|---|
| T1.1 | Spike code hardened: tests, docstrings, D07 v2 output with run id and context record | Planner regression on the golden job |
| T1.2 | Mode C planning and 7-joint segments; positioner speed ω = travel speed / radius | Sim: J1 100 % with clearance report |
| T1.3 | J2 robot-only: 100 % or rejected with failing points, colliding link / object and C8 coverage (B-12) | Sim: J2 result reproducible |
| T1.4 | Mode B with **index angles entered by the Engineer** — only if Go and J2 robot-only < 100 % (B-12). Automatic Mode B search = R3 | Sim: coverage with entered angles |
| T1.5 | Approach / retract, OMPL air and index moves planned once and stored; dense check with the displacement bound (B-03 §7) | Sim: C4 ≥ 10 mm unpadded |
| T1.6 | Time parameterisation (A47), wrist-3 on unwrapped values (A48) | Speed error < 5 % |
| T1.7 | Reproducibility as defined in B-11: weld / approach / retract payloads identical within one image digest; air moves compared by verdict and cycle time ±10 % | Regression test |
| T1.8 | Batched in-process `SceneBackend` — **only if** C5 > 30 s per seam (B-06) | C5 re-measured |

### WP2 — Engineering chain (9–20 Nov)

| # | Task | Test |
|---|---|---|
| T2.1 | `awm_cad`: D20 view model complete, collision mesh with `face_id`, joint edges flagged with their solid pair (B-02) | Golden part: edge ids stable across re-imports |
| T2.2 | `awm_seam`: picked edge on either solid → joint edge; D04 v2 with source-solid ids | J1, J2 picked from their edge ids |
| T2.3 | `awm_process`: WPS v2 with `filler_wire`, `wire_side`, `psi_window_deg`, `tip_type` (Assumption until I5) | pytest |
| T2.4 | Placement D19 = placement of the golden part in the cell CAD, `source: fixture_cad`, `verified: false`, refused by every non-simulation gate (B-12). A17 study after Sim-1 | pytest |
| T2.5 | `awm_program`: 7-joint segments, events at rest of robot and positioner, context record | Program build (Sim) |
| T2.6 | `awm_verify`: frozen per-revision bundle `programs/<rev>/{bundle/, release.json}`, active pointer per scope (B-09); M7 rebuilds the scene from the bundle, C1 self-test, per-segment minimum distance and closest pair; simulation-scope release | Sim e2e: golden part → PASS → simulation release; one changed byte → refused |
| T2.7 | Planning-run runner (B-11): `awm_api` starts the CLIs as subprocesses; run folder; one slot per cell (409 when busy); cancel = SIGTERM; no rclpy in `awm_api` | pytest with FastAPI test client |

### WP3 — Web UI, staged (2–20 Nov; starts on the example files of T0.5)

| # | Task | Test |
|---|---|---|
| T3.1 | `awm_api`: `/api/v1`, RFC 9457 errors; Sim-1 endpoints only: `/auth/login`, `/jobs`, `/jobs/{id}/view`, `/seams`, `/placement` GET, `/plan`, `/runs/{run_id}`, `/simulation`, `/release` (simulation scope); credentials in a git-ignored secrets file | API tests |
| T3.2 | `awm_web`: **S1-min** (list / create jobs), **S2** (picking, chaining, attributes, seam frame + beam axis drawn every 90°), **S4-lite** (plan with progress, robot + positioner playback, point colours with colliding link / object, cycle time) | Component tests + one scripted browser run |
| T3.3 | `web` launch | Smoke test |

Without screens in Phase 1 (CLI / API): S3, S5, S7, S8, S9, S10. S6 at HW-1. Configuration / ψ overrides and the
placement study edited in JSON until R3.

### WP4 — Minimal runtime on URSim (23 Nov – 4 Dec, B-19)

| # | Task | Test |
|---|---|---|
| T4.1 | `awm_motion` package (FK, D09 → JointTrajectory, override time scaling, start-state check, dense check); dependency rule in CI (B-06) | CI import check |
| T4.2 | D09 = exact commanded trajectory per controller: positions, velocities, accelerations, times (B-15) | M7 verifies the interpolated motion |
| T4.3 | M10 sim, table-driven from the DES-002 weld-event behaviour table; no automatic emission retry; heartbeat counter driven by M8 (B-17, B-18) | pytest per table row |
| T4.4 | M9 in **Python** (ADR-08 modified): replays D09 verbatim; start-state check, speed-scaling monitor, positioner tracking | Sim-2 replay of the golden part |
| T4.5 | M8-minimal: job load through the active pointer + context compare; scope table (simulation now; trial / production **[E-02]**); mode derived from the connected drivers, any mix → NOT_READY; seam loop; FAULT; JSON-lines start / end records per seam (M11 / M12 deferred, B-24); manual recovery with the teach pendant | Sim-2 e2e |
| T4.6 | Acceptance script (B-19 §6): replay; start refused < 100 % scaling; scaling change in a weld → WeldEnd + FAULT; LaserReady timeout → EmissionOff → FAULT, no retry; EmissionLost; Mode C stall; M8 tick stopped → emission off; door / LightWELD / gas / wire flags; mixed mode; one changed byte in D09; commanded vs actual | Independent run by the commissioning engineer **[E-03]** |

## 4. Schedule

| Dates | Work |
|---|---|
| Mon 2 – Thu 5 Nov | WP-S spike (S.1–S.7); T0.5 contract freeze on Wed 4 Nov; WP3 starts on the example files |
| Fri 6 Nov | TST-002 decision **[E-01]**; DES-001 Rev 0.6 issued; OI-27 answered |
| 9 – 13 Nov | T0.1–T0.4, T0.6, T0.7; WP1 T1.1–T1.3; T2.1–T2.3; J1 Mode C by CLI on Fri 13 Nov; customer demo agreement (B-22) |
| 16 – 20 Nov | T1.4–T1.7, T2.4–T2.7, T3.1–T3.3; Sim-1 demo on Fri 20 Nov; DES-001 Rev 0.7 at the Sim-1 exit |
| 23 Nov – 4 Dec | WP4; ADR-23 review gate active from 23 Nov; Sim-2 on Fri 4 Dec |

Spike outcomes (B-01 §5):
- **Go** → WP1 continues 9 Nov.
- **Pivot** → at most 3 days of TST-002 §7 steps 1–3 in MoveIt; J2 enters Sim-1 as "rejected with failing points";
  Tesseract evaluation after Sim-2.
- **No-Go** → Sim-1 = J1 Mode C only; Phase 1 scope escalated to Linh Tran / Tac Huynh.
- **Inputs or tooling missing on Thu 5 Nov** → "not decided", at most +3 working days, not Pivot.

Capacity rule: accepted effort per stage ≤ 70 % of its working days (B-14). If WP3 is late, Sim-1 is shown with the
same API and RViz playback.

## 5. Inputs needed (owner, due)

| # | Input | Needed for | Owner | Due |
|---|---|---|---|---|
| I1 | `REF_*` markers in the Fusion model; remove the duplicate `ASSY, Welding tool`; state the up axis (OI-23) | T0.2 (until then Assumption) | Nghia Nguyen | before HW-1 |
| I2 | Nozzle tip / TCP point and tip shape of the LightWELD head | S.4, T0.3 | Khang Doan | 2026-11-04 (else TST-002 §3.1 Ø 20 mm nozzle) |
| I3 | Holders that rotate with the part; positioner angle limits; r_max of the rotating set (OI-27) | S.6, T1.2 | Mechanical | 2026-11-06 |
| I4 | Positioner drive, encoder, fieldbus (OI-14); drive requirement per ADR-22 | HW-1 | Mechanical / electrical | 2026-11-20 |
| I5 | J1 / J2 WPS: filler wire yes / no and side, ψ window, travel speed, preset id **[E-05]** | S.5, T2.3 | Welding engineer via Truong | **2026-11-04** |
| I6 | Upper laser enclosure model (OI-24) | T0.2 | Mechanical | before HW-2 |
| I7 | Company Git remote + CI runner **[E-06]** | T0.7 | Truong | 2026-11-13 |
| I8 | Named PLC & safety engineer, commissioning engineer, ADR-23 reviewers **[E-03]** | WP4, HW-1 | Linh Tran | 2026-11-20 |
| I9 | Customer demonstration agreement; demo part 564082 or 152056 **[E-04, E-08]** | Sim-1 scope | Tac Huynh | 2026-11-13 |

## 6. Rules carried over

No part-specific code: J1 / J2, component names and poses live only in job / cell data and config. No partial-path
fallback: a seam is accepted only at 100 %. Never connect to a real robot, PLC, LightWELD or positioner drive; URSim
and mock hardware only. No emission command outside the PLC sim mode; the software agent writes PLC requirements
(DES-002 table), never the PLC program. Units only through `awm_core.units`. Code and wiki changes never in the same
commit.

## 7. Open points to confirm in the review

1. TST-002 decision on Fri 6 Nov with the frozen inputs of TST-002 Rev 0.3 (E-01).
2. Filler wire on J1 / J2 (E-05) by Wed 4 Nov — it fixes the ψ case that the decision covers.
3. Phase 1 deliverable on 4 Dec = Sim-2 exit; HW-1 / HW-2 dated later in PM-004 (E-04).
4. Python M9 for Phase 1 (ADR-08 modified, B-19).
5. Contract freeze review (one hour) on Wed 4 Nov.
