# PROGRESS

Read this first after a restart. Continue from the first open task. Plan: `PLAN.md`.

## Rev 0.4 migration (from 2026-10-27)

| Step | Status | Notes |
|---|---|---|
| Sync design baseline (DES-001 / DES-003 Rev 0.4, TST-002 Rev 0.2, CLAUDE.md) | done | 2026-10-27 |
| Decisions CP-9…CP-23 (Z up, TCP, Sim-1 end, placement, calibration, tacks, positioner, Fusion CAD) | done | `docs/generated/DES-001_change_proposal.md` |
| Cell CAD read test (Fusion STEP, golden part assembly, positioner) | done | findings in CP-11, CP-20, CP-23 |
| PLAN.md Rev 0.4 | superseded by PLAN.md Rev 0.6 | 2026-10-28 |
| DES-001 Rev 0.6 draft + AI review board (B-01…B-25), TST-002 Rev 0.3 draft, PLAN.md Rev 0.6 | done, waiting for Truong's review; escalations E-01…E-08 open | commit a96c188, 2026-11-02 |
| WP-S S.1 joint-aware seam normals (`awm_geometry.joints`, `awm_geometry.assembly`) | done — **G1 PASS** on the golden part (0.000° vs 45° per-solid) | 2026-11-02, `test_joints_occ.py` |
| Agent team set up (`docs/generated/AGENT_TEAM.md`, `.claude/agents/`, `scripts/wt`, per-checkout `ROS_DOMAIN_ID` in `scripts/dx`) | done | 2026-11-02, accepted by Truong |
| WP-S S.2 M4 core (`awm_core.units`, KinematicsBackend ur-analytic-ik 0.1.0.post3, SceneBackend move_group) | done — merged d031a60…47532f2; rules audit PASS, review no blocker / major | 2026-11-02, plan `docs/superpowers/plans/2026-11-02-ws-s2-m4-core.md`; image not rebuilt |
| S.2 review follow-ups (minor): R1 IK branch label per solution (needed by S.3 ladder edges); R2 `ik()` can return [] exactly at q5 = 0 → report "singular", not "unreachable"; R3 `check_rigid` 1e-9 rejects 6-decimal rotations → precision / quaternion rule in T0.5; R4 test pinning fk_tcp composition order; R5 C1 must cover collision correctness; R6 pin scikit-build-core / nanobind; R7 `apt-get update` in docker/README; R8 Assumption: nominal DH = nominal URDF, calibrated kinematics_params at HW stage | open — R1, R2, R4 in S.3; R3 in T0.5; R6, R7 cell-sim; R8 change proposal | 2026-11-02 |
| D20 view model contract v2.0 (T0.5 part) + exporter `export-view` + `import --component` (cad-geometry, wp0-d20-export) | done — merged 31ad990 on Truong's choice; review findings 1–9 fixed, rules audit PASS | 2026-11-02, `docs/generated/contracts/D20_view_model.md`; golden part: 3 solids, 4 closed joints (J1 = JE2, J2 = JE4 outer Ø26.67; JE1 / JE3 bore Ø20.93, kept) |
| S2-lite read-only D20 viewer `awm_web` (web, wp3-s2-lite-viewer) | done — merged ed8fa57 on Truong's choice; review fixes R1–R5 done, rules audit PASS | 2026-11-02, `src/awm_web/README.md`; visual check by Truong pending |
| D20 follow-ups: N1 **major** record `component` in part.json v2, export-view reads / checks it (before T2.2 uses D20 ids); edge in two joints → contract change to a list (three-plate corners); N2 test of name mapping through ShapeFix history; N3 whole-file export loses solid names; N4 private `topology._*_NAMES` used in view.py; old `occ_io.MM_TO_M` / `math.radians` → `awm_core.units` (T0.1) | open — N1 lead in T0.5 | 2026-11-02 |
| Docker Desktop host networking enabled, awm_dev restarted: servers in awm_dev are reachable from WSL and Windows (viewer on port 5174) | done | 2026-11-02 |
| T0.5 contract freeze (lead): v2 schemas D03 (+component, closes N1), D04, D07, D09, D10, active, D19 + J1 Mode C examples; Sim-1 OpenAPI | draft done — d718396, 50f5d75; waiting for Truong's one-hour review (Wed 2026-11-04); `awm_interfaces` v1 still to draft (needed by WP4) | `docs/generated/contracts/T0.5_contract_freeze_v2.md` |
| WP-S S.3 (planner, ws-s3-state-space, 7cb67a2) | done — review PASS, review fixes done; **merged 2026-11-04 with S.4 C (6e3e9f5)**; CP-53 (Engineer-entered arcs, J2 = 2 arcs), F1 ceiling, F6 wording | plan `docs/superpowers/plans/2026-11-02-ws-s3-state-space.md` |
| WP-S S.4 (plan `docs/superpowers/plans/2026-11-03-ws-s4-scene-clearance-contact.md`; H-BOX first, sequential, ≤ 2 agents — Truong 2026-11-03) | A (cad-geometry, ws-s4a-scene-export): done, **merged 2026-11-03 (21333a1)** — hull/mesh per component, export guard (Part3 0.923 mm ≤ 1.0), cell_scene v2, face retry with nodes_only (D10), peak RAM 1.5 GB; last fix round (M1/M2, a1b8214..b8cd6f7) written by the lead while agents were rate-limited, not independently reviewed yet; T0.2 open items in D10; B (cell-sim, ws-s4b-cell-model, 1262a20): done, review fixes done (positioner-angle guard), re-run 216 tests 0 fail — ready, **merged 2026-11-03 (ad60793)**; C (planner, ws-s4c-checks): done — C1–C5, G2 run 4 PASS on a clean tree (10/10, 10/10, 5/5) **accepted by Truong 2026-11-04**; review no blocker, fix round re-reviewed OK, rules audit PASS; **merged 2026-11-04 (6e3e9f5, branch at c2fd5bc)**; open items in `docs/generated/spike/s4_open_items.md` (m6 clearance link set awaits Truong). T0.2 follow-ups (ws-s4a-t02-fixes: lock, recovery, F1–F3) re-reviewed OK, **merged 2026-11-04 (53c51b8)**; minor left: mixed cell/job state if the undo rename itself fails, weak F3 test assertion | CP-54 S10-lite, CP-55 scene objects, scene decisions D1–D8; awm_dev restarted after WSL drop 2026-11-03 |
| WP-S S.5 … S.7 (sweep J2 2 arcs, J1 Mode C check, records) | S.5 done (73aa0eb, not reviewed / merged): J2 robot-only **C2 FAIL** (arc 1 0 %, arc 2 38.3 %; ~40 mm no IK, F4 part / holder Part2:1), C5 557 s → Pivot per TST-002 §6 (meeting 6 Nov). **Truong 2026-11-04: run Mode B with 2–3 Engineer-entered positioner angles on Thu 5 Nov** (pulled forward from T1.4 as Pivot evidence). Lead: CP-53 pitch refinement applied to scene-valid nodes only (proposed, no objection). S.6 done (a45ca60): J1 Mode C F4 / clearance PASS (≥ 20 mm), F9 fails (Ø 20 tip on thread crests). Review of S.5 / S.6: no blocker; M1 present J2 fail as "θ 0 + H-BOX + Ø 20", M2 C5 not a valid measurement, M3 F9 rule (b) chamfer artefact → **CP-56 (lead, delegated by Truong 2026-11-04)**: rule (b) only faces rising towards the nozzle; results per nozzle Ø 20 / 12 / 8. J2 Mode B screening running | decision Fri 2026-11-06 |
| WP0 … WP4 | not started | — |

## Rev 0.2 offline chain (2026-10-19, superseded by the table above)

| Phase / task | Status | Notes |
|---|---|---|
| Phase -1 environment (awm_dev, scripts, Dockerfile, env report, change proposal) | done | commit 14252f5 |
| Phase 0 discovery (demo map) | done | docs/generated/demo_discovery.md |
| Phase 1 plan | done | PLAN.md |
| T1 awm_core | done | 22 tests (ROS py3.12 + conda py3.10) |
| T2 awm_geometry import/edges/test parts | done | OCC 8 API; explicit mm→m; BRepOffset_Analyse convexity |
| T3 awm_geometry sampling | done | exact curve evaluation; colcon runs OCC suite via conda (12 tests) |
| T4 awm_cad | done | mocked + real OCC import test |
| T5 awm_seam | done | picking, chain, attributes, export, rematch (16 tests) |
| T6 awm_process | done | WPS lookup, SAMPLE gate, process.json (7 tests) |
| T7 awm_interfaces | done | builds; §7.1 v0 |
| T8 awm_description | done | xacro + SRDF + MoveIt cfg, mock hw only (6 tests) |
| T9 awm_bringup | done | sim.launch.py headless, D16 params, repo template, SimLaunch test helper (7 tests) |
| T10 awm_path pure | done | torch axis, tool roll, approach/retract (10 tests) |
| T11 awm_path MoveIt | done | plate 2/2, pipe 2 arcs 2/2, unreachable + TCP-mismatch negative tests (sim) |
| T12 awm_program pure | done | constant-speed + trapezoid timing (6 tests) |
| T13 awm_program MoveIt | done | stored joint path reuse + FK check, events, stale-calibration test (4 sim tests) |
| T14 awm_verify | done | static checks, replay on mock hw, records, release + lock (9 tests) |
| T15 end-to-end | done | run_chain: plate (2 seams) + pipe (3 arcs) PASS + released (simulation); 360 deg seam rejected at A4 (3 e2e tests) |
| T16 docs | done | READMEs, USER_GUIDE, TRACEABILITY, FINAL_REPORT, change proposals CP-5..8 |

## Last error
none (resolved: pick_ik global mode + cost settings, see decision log)

## Decision log
- 2026-10-19: awm_dev runs as uid 1000 (image user `ubuntu`), HOME=/tmp/awm_home; no fix_owner step needed.
- 2026-10-19: demo entrypoint bypassed (forces ROS_DOMAIN_ID=0); demo FastDDS profile unset in scripts/dx.
- 2026-10-19: git identity set repo-local (Truong Hoang) because none was configured.
- 2026-10-19: pick_ik `mode: global`, `minimal_displacement_weight: 0`, `cost_threshold: 0.001`: with local mode or a displacement weight no IK was ever found for poses far from the seed.
- 2026-10-19: full 360 deg pipe seam is NOT reachable from the fixed UR5e with the 45 deg demo torch (reach map: CCW fails -150..-135 deg, CW fails 135..165 deg; tool0 passes over the base axis). Implemented A22 start / end as optional `range_m` partial seams (schema v1 optional field, OI-7) and weld the pipe as two arcs. Moving the fixture does not fix it (x=0.47/0.50 tested).
- 2026-10-19: `tool_roll` parameter (travel | up); default `travel` (x = travel direction). `up` gave worse reach on the pipe.
- 2026-10-19: test part pick on a cylinder seam vertex is refused as ambiguous (by design); pipe pick moved to 90 deg.
- 2026-10-19: awm_dev has no init process: stopped launches leave <defunct> zombies. Harmless; add `--init` when the container is next recreated (docker/README.md).
- 2026-10-19: velocity limits per segment kind: weld segments (WPS travel speed) are checked against the configured joint limits `max_joint_velocity_rad_s`; air / approach / retract against those limits x `velocity_scaling` (0.2). No limit raised. Pipe S01 at 8 mm/s needs wrist_2 0.91 rad/s (29 % of 3.14 rad/s).
- 2026-10-19: pick_ik global mode is stochastic (also inside compute_cartesian_path), so re-solving in M6 could fail or pick another branch. M4 stores the checked joint path in path.json (`joint_path`, D07 "joint path" per DES-003 A44 output), valid only for the calibration hash recorded there. M6 reuses it, verifies every weld point by FK against the transformed seam (A61, 0.2 mm), and times approach / retract itself (trapezoid within scaled limits).
- 2026-10-19: IK solvers: group `weld_arm` (Cartesian paths, OMPL) uses KDL (deterministic, tracks 2 mm steps); group `weld_arm_search` (same chain) uses pick_ik global, only to find IK branches of approach poses. pick_ik global/local for Cartesian gave 2/3..4/5 success on the pipe arcs; KDL 10/10. Pipe welded as 3 arcs (S01 CCW -120..60, S02 CCW 60..180, S03 CW -120..180 deg).
- 2026-10-19: OMPL air moves can be rejected by move_group ValidateSolution (torch vs fixture between states); `longest_valid_segment_fraction` 0.005 -> 0.002 and `air_planning_retries` 3.
- 2026-10-19: full `colcon test` (all packages in parallel) failed once: the A71 dense check (0.02 rad) found torch<->part contact inside an OMPL air move that move_group's ValidateSolution had accepted (it checks the time-parameterised waypoints only). Fix: M6 runs the same dense check on every air move and replans (`air_planning_retries`); OMPL `longest_valid_segment_fraction` 0.002 -> 0.0005. The shared check lives in awm_path.motion (`first_collision`).
