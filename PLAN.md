# APLSMN601 — implementation plan Rev 0.4 (Sim-0 → Sim-1 → Sim-2)

Status: **draft for review by Truong Hoang**, written by the software agent on 2026-10-27. No code has been changed
for this plan yet. The previous plan (offline chain A1–A7 on DES-001 Rev 0.2, delivered 2026-10-19) is in git:
`git show 13ef478:PLAN.md`.

Sources: DES-001 Rev 0.4 (software architecture), DES-003 Rev 0.4 (function register, "Sim stage" column),
TST-002 Rev 0.2 (collision-avoidance spike), change proposal `docs/generated/DES-001_change_proposal.md` CP-9…CP-23
(decisions of 2026-10-27), cell CAD `docs/3D/APLSMN601_Automated Weldment System.step`.

## 1. Goal per milestone

| Milestone | Date | Done when |
|---|---|---|
| TST-002 decision | Thu 2026-10-29 | Spike results C1, C2, C3, C5, C8 on the golden part (J1, J2), Go / Pivot / No-Go input |
| Sim-0 | Fri 2026-10-30 | mm units in all project data, cell imported from the Fusion STEP, cell model with LightWELD head and positioner joint in MoveIt, `awm_interfaces` contract in mm reviewed |
| Sim-1 | Fri 2026-11-20 | In the browser: create job from the golden part → pick J1, J2 → plan (ψ × IK ladder, positioner modes C / B) → failing points red → playback of robot + positioner (mock hardware) → simulation report D15. Sim-1 ends at A71 (CP-17) |
| Sim-2 | Fri 2026-12-04 | Same program, released with `scope: simulation`, replayed by the runtime chain (M8 / M9 / M10 sim) through `ur_robot_driver` on URSim + mock positioner joint |

Golden part: `564082-Assembly-3 parts` (pipe nipple + solid + elbow) from the cell CAD. Two closed butt joints,
Ø 26.67 mm: **J1** nipple ↔ solid, coaxial with the positioner axis → Mode C; **J2** solid ↔ elbow, 76.6 mm off the
axis, axis perpendicular to it → Mode B or robot alone.

## 2. What changes against the delivered code (Rev 0.2)

| Area | Delivered 2026-10-19 | Rev 0.4 + decisions of 2026-10-27 | Action |
|---|---|---|---|
| Units | SI (m, rad) in schemas | mm / deg / s in all project data; one adapter (ADR-14) | Migrate (T0.1) |
| Process | MIG torch, weave | LightWELD laser, presets, wobble = head parameter | Rewrite `awm_process`, tool model |
| Tool model | Demo MIG torch, nominal | LightWELD head + mount from Fusion CAD, TCP at nozzle tip (CP-10) | New (T0.3) |
| Cell model | Hand-written boxes | Fusion STEP assembly, marker components `REF_*` (CP-23) | New import (T0.2) |
| Positioner | none | 1-axis, Phase 1, ROS 2 control (CP-20) | New joint + controller (T0.3, T0.4) |
| IK / planner | KDL Cartesian + pick_ik seeds | Analytic UR IK, 8 branches × ψ, ladder-graph DP (ADR-12), Mode C / B | Rewrite `awm_path` core (WP1) |
| Seams | edge ids, attributes, range_m | + n1 / n2, closed flag, sequence (no tacks, CP-19), Engineer only (CP-21) | Extend `awm_seam` |
| CAD outputs | STL mesh, edges.json | + D20 `view.glb` + `topology.json`, collision mesh with `face_id`, multi-solid | Extend `awm_geometry` / `awm_cad` |
| Placement | fixed in calibration | D19 with `source` study / fixture_cad (CP-22) | New |
| Program | 6 joints, MIG events | 7 joints, positioner segments, LaserReady / EmissionOff | Extend `awm_program` |
| Verify / release | simulation / production scopes | same + D19 source rule, positioner in dense check | Extend `awm_verify` |
| UI | none (CLI + RViz) | Web: `awm_api` (FastAPI) + `awm_web` (React / r3f) | New (WP3) |
| Runtime | none | M8 / M9 / M10-sim for Sim-2 | New (WP4) |

Kept as is: job repository, headers, SHA-256 stale-input checks, OCC subprocess bridge, simulation / production
release scopes, dense collision check (CP-16), Docker workflow.

## 3. Work packages

Each task lists its test. "Sim" = needs the ROS sim launch; everything else runs with plain pytest.

### WP0 — Foundations (Sim-0)

| # | Task | Test |
|---|---|---|
| T0.1 | **mm migration.** `awm_core.units`: the only mm ↔ m / deg ↔ rad converter (scalars, vectors, poses, joint lists). Schemas v2 in mm / deg (`*_mm`, `*_deg`, `*_mm_s`); readers refuse v1. All packages switched; URDF / MoveIt calls go through the adapter. Static check: no `1000` / `0.001` scale literals outside `awm_core.units` | Adapter round-trip tests; schema tests; source-scan test fails on ad-hoc scaling |
| T0.2 | **Cell CAD import** (`awm_geometry cell-import`): read the Fusion STEP with XCAF; CAD → `world` rotation from a config value (this file: Y up); component → role map in `cell/cad_map.yaml` (data, not code): `static`, `positioner_rotating`, `fixture`, `part`, `head`, `mount`, `ignore`; collision meshes (convex hulls for the surface-model head), frames from `REF_*` markers; refuse import on missing markers, duplicates or unit mismatch. Extract the part sub-assembly to the job (`model.step` + source file SHA-256 + component path) | pytest on the real file (slow mark, 60 s read) + a small synthetic assembly |
| T0.3 | **Cell model v2** (`awm_description`): UR5e from `ur_description`; LightWELD head + mount links, `nozzle_tip` / `nozzle_body` split, `tcp_laser` (CP-10); `positioner_joint` (revolute, limits from config) with rotating holders and the part as children of `positioner_flange`; static cell meshes from T0.2 loaded from the repository root (cell data stays out of the source tree); SRDF + ACM (only `nozzle_tip` ↔ part); padding 3 mm | xacro → URDF parse; FK of `tcp_laser` vs CAD pose; ACM test |
| T0.4 | **ros2_control**: one controller manager, `ur_manipulator` controller + `positioner_trajectory_controller`; `mock_components` for both in Sim-1. Launch names `sim_mock` (Sim-1), `sim_ursim` (Sim-2) (CP-14 #5) | Launch smoke test: both controllers active, `/joint_states` has 7 joints |
| T0.5 | **`awm_interfaces` v1**: lengths `*_mm`, LaserReady / EmissionOff, positioner fields; reviewed before WP4 | colcon build + message round trip |
| T0.6 | **Environment**: `ur-analytic-ik` in a venv inside `awm_dev` (PEP 668), `moveit_py` check, Node.js for `awm_web`, all recorded in `docker/Dockerfile` | Version report in `docs/generated/env_report.md` |

### WP1 — Planner (ADR-12, TST-002) — first results for 29 Oct, complete for Sim-1

| # | Task | Test |
|---|---|---|
| T1.1 | Seam frame (CP-9): X = tangent, Z = bisector of n1 / n2 out of the material (butt joint on a tube: n1 = n2 → surface normal), beam axis = −Z; arc-length sampling 1–3 mm; 360° closing overlap | Pure pytest on J1 / J2 geometry (frames orthonormal, Z outward) |
| T1.2 | Analytic IK: all 8 UR5e solutions for a `tool0` pose (ur-analytic-ik; fallback: port of `ur_kin.cpp`), joint limits, singularity measures | Compare with MoveIt FK on random states (< 0.01 mm) |
| T1.3 | Candidate sweep per point: ψ 0–350° step 10° × 8 branches (× positioner angle in Mode B); validity + minimum distance from MoveIt (in-process `moveit_py` planning scene if available, else services); output `candidates.csv` (TST-002 format) | Sim: C1 scene sanity 10 / 10 colliding + 10 / 10 free |
| T1.4 | Ladder-graph DP: one IK branch per seam, \|Δq\| ≤ 5° per step, wrist-3 within ±180° of start and ±270° absolute (CP-14 #3), cost = joint motion + deviation from nominal + low-clearance penalty + wrist-3 travel; per-point status (OK / collision / reach / singularity / limit) for the red points | Pure pytest on synthetic ladders; Sim: C2, C3 on J2 |
| T1.5 | **Mode C** (seam coaxial with the positioner axis, detected from geometry): one fixed robot pose (ψ / branch with the largest clearance over the full turn), collision of head vs. rotating part + holders every 2° over 360° + overlap; positioner speed ω = travel speed / radius | Sim: J1 100 % with clearance report |
| T1.6 | **Mode B**: split the seam into arcs, candidate positioner angles (e.g. 15° steps), ladder per (arc, angle), fewest segments covering 100 %; robot-only first, positioner only if needed | Sim: J2 coverage, C8 numbers robot-only vs. with positioner |
| T1.7 | Approach / retract 30–50 mm along the beam axis; OMPL air moves (robot) and index moves (positioner, robot parked) with dense collision check (CP-16) | Sim: C4 clearance ≥ 10 mm |
| T1.8 | Time parameterisation (A47): robot weld speed in the part frame, error < 5 %; Mode C positioner profile; wrist-3 / cable-wrap check (A48) | Pure pytest on speed error |
| T1.9 | `path.json` v2 (D07): poses in mm in the `part` frame, joint path (6 + positioner, deg), mode per segment, per-point status, D08 hash (CP-15, CP-18). TST-002 harness: C1–C6, C8 table for TST-002 Rev 0.3 | Schema test; harness run on the golden part |

### WP2 — Engineering chain on Rev 0.4 (Sim-1)

| # | Task | Test |
|---|---|---|
| T2.1 | `awm_cad`: split solids, D20 view model (`view.glb`, one node per face / edge, chord ≤ 0.05 mm; `topology.json`), collision mesh with `face_id` (chord ≤ 0.1 mm near seams) | pytest on the golden part: 3 solids, edge ids stable across re-imports |
| T2.2 | `awm_seam`: pick by edge id, auto-chain tangent edges, attributes incl. closed flag and sequence, n1 / n2 export, fingerprint re-match | pytest: J1, J2 picked from their edge ids |
| T2.3 | `awm_process`: WPS library v2 (mm, LightWELD preset id, CW / HPP, wobble display, focus offset, versioning, released WPS read-only); one SAMPLE WPS | pytest |
| T2.4 | Placement (CP-22): D19 with `source`; Sim-1 placement study = candidates entered by hand, each planned in full; import from the fixture CAD (T0.2) | pytest + Sim: golden part from the cell CAD |
| T2.5 | `awm_program`: 7-joint segments (air, approach, weld_robot, weld_positioner, retract, index), events at rest of robot **and** positioner, calibration hash | Pure pytest + Sim program build |
| T2.6 | `awm_verify`: replay of both controllers on mock hardware, dense check incl. positioner, D15 report, simulation-scope release (CP-17) and D19 source rule | Sim e2e: golden part → PASS → simulation release |

### WP3 — Web UI (Sim-1), DES-001 §4.5 / §7.4

| # | Task | Test |
|---|---|---|
| T3.1 | `awm_api` (FastAPI + rclpy): `/auth/login` (single local login, roles stubbed per A88), `/jobs`, `/jobs/{id}/view`, `/seams` (Engineer only), `/process`, `/plan` (async + WebSocket progress), `/simulation`, `/release` (simulation scope) | pytest with FastAPI test client |
| T3.2 | `awm_web` (React + TS + react-three-fiber + urdf-loader): S1 job library, S2 seam picker (edge picking from D20, chain, attributes), S4 simulation (scrubber, red points, robot + positioner animation, cycle time); S3 process minimal | Component tests + one scripted browser run |
| T3.3 | `web` launch: `awm_api` + static front-end build | Smoke test |

### WP4 — Runtime chain on URSim (Sim-2)

| # | Task | Test |
|---|---|---|
| T4.1 | URSim e-Series container on a private Docker network (no host network, no real IP), `sim_ursim` launch: `ur_robot_driver` + mock positioner joint in the same controller manager | Driver connects, 7 joints in `/joint_states` |
| T4.2 | `awm_plc_bridge` sim mode: WeldStart / WeldEnd → simulated LaserReady / EmissionOff, timeouts, heartbeat; no emission command path to hardware | pytest |
| T4.3 | `awm_executor` (M9, C++ per ADR-08): start-state check, segment replay, speed-scaling monitor (weld only at 100 %) | Sim-2 replay of the golden part |
| T4.4 | `awm_cell` (M8, minimal): load release (scope check, hashes), seam loop, FAULT on timeout / scaling change | Sim-2 e2e + fault injection |

## 4. Schedule (proposal)

| Dates | Work |
|---|---|
| Wed 28 – Thu 29 Oct | Spike for TST-002 on J1 / J2: T1.1, T1.2, T1.3, T1.4 on a provisional scene (cell CAD as static meshes, head from CAD, positioner fixed at 0° plus a Mode C check) → C1, C2, C3, C5, C8 numbers for the 29 Oct meeting |
| Fri 30 Oct (Sim-0) | T0.5 contract, T0.6 environment; T0.1 started |
| 2 – 6 Nov | T0.1 – T0.4 finished; T1.5 – T1.9 |
| 9 – 13 Nov | WP2; T3.1 |
| 16 – 20 Nov | T3.2, T3.3; Sim-1 demo on Fri 20 Nov |
| 23 Nov – 4 Dec | WP4; Sim-2 on Fri 4 Dec |

**Risk to the dates:** Sim-0 by 30 Oct is not realistic for T0.1 – T0.4 together with the spike; the plan keeps the
spike first and moves the rest of WP0 into the week of 2 Nov. The Web UI (WP3) is the largest new block; if it is
late, Sim-1 is shown with the same API and RViz playback, and the browser screens follow.

## 5. Inputs needed (owner)

| # | Input | Needed for | Owner |
|---|---|---|---|
| I1 | Marker components `REF_WORLD`, `REF_UR_BASE`, `REF_FIXTURE`, `REF_PART`, `REF_POSITIONER`, `REF_TOOL0`, `REF_TCP` in the Fusion model; remove the duplicate `ASSY, Welding tool`; state the up axis | T0.2, T0.3 (until then frames are derived from components and marked Assumption) | Mechanical (Nghia Nguyen) |
| I2 | Nozzle tip / TCP point of the LightWELD head (IPG data or measured on the CAD) | T0.3, CP-10 | Khang Doan |
| I3 | Which holders rotate with the part (`Part1`, `Part2`, `Part3`)? Positioner angle limits (cable / belt) | T0.3, T1.5 | Mechanical |
| I4 | Positioner motor driver, encoder, fieldbus (EtherCAT proposed) | Hardware stage only; Sim uses mock | Mechanical / electrical (OI-14) |
| I5 | Sample WPS for J1 / J2: travel speed, head angles, preset id | T1.5, T1.8, T2.3 | Welding engineer |
| I6 | Upper laser enclosure model (collision when the robot reaches high) | T0.2 | Mechanical |
| I7 | CI host for Sim-0 "Git + CI" (GitHub Actions needs a public base image, not the local `ros2-jazzy-hardware-accelerated`) | Sim-0 | Truong |

## 6. Rules carried over

No part-specific code: J1 / J2, component names and poses live only in job / cell data and config. No partial-path
fallback: a seam is accepted only at 100 %. Never connect to a real robot, PLC, LightWELD or positioner drive; URSim
and mock hardware only. No emission command outside the PLC sim mode. Units only through `awm_core.units`. Code and
wiki changes never in the same commit.

## 7. Open points to confirm in the review

1. Spike first, Sim-0 rest in the week of 2 Nov (§4).
2. Mode B for J2: try robot-only first, use positioner angles only when robot-only is below 100 % (T1.6).
3. Cell data location: generated cell meshes and `cad_map.yaml` under `<repo_root>/cell/model/`, not in the
   `awm_description` source tree (T0.3).
4. Web UI fallback for Sim-1 if WP3 is late (§4).
5. M9 in C++ as ADR-08 says, or Python first for Sim-2 and C++ later.
