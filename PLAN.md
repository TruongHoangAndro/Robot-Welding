# APLSMN601 offline engineering chain (A1, A2, A3, A4, A6, A7) — implementation plan

Status: Phase 1 plan, written by the agent on 2026-10-19. Decisions are logged in §8 and in `PROGRESS.md`.
Sources: DES-001 Rev 0.2 (+ `docs/generated/DES-001_change_proposal.md`), DES-003 (exported in
`docs/generated/function_register_*.md`), demo discovery `docs/generated/demo_discovery.md`.

## 1. Packages and dependency graph

```
awm_interfaces (msg/srv/action, §7.1, v0 — not used by the offline chain yet)

awm_core  (pure Python: numpy + jsonschema; importable from ROS python 3.12 AND conda python 3.10)
   ▲   ▲   ▲
   │   │   └──────────────────────────────────────────────┐
awm_geometry (pure Python + OCC, conda env only; CLI)      │
   ▲ (subprocess, occ_python_cmd)                          │
awm_cad (A11–A14 wrapper) ─► awm_seam (A21/A22/A24, pure) ─► awm_process (A31–A33, pure)
                                                           │
awm_description (D14: xacro, SRDF, MoveIt cfg) ◄── awm_bringup (sim.launch.py, D16 config, repo template)
                                                           │
awm_path (A41–A45, rclpy client of move_group) ─► awm_program (A61–A64) ─► awm_verify (A71–A74, run_chain)
```

| Package | Type | Depends on | Runs in |
|---|---|---|---|
| awm_interfaces | ament_cmake | rosidl | ROS |
| awm_core | ament_python | numpy, jsonschema | ROS py3.12 and conda py3.10 |
| awm_geometry | ament_python (no rclpy) | awm_core, OCC | conda py3.10 only (installed by colcon only to be locatable) |
| awm_cad | ament_python | awm_core, awm_geometry (subprocess) | ROS |
| awm_seam | ament_python | awm_core | ROS / plain python |
| awm_process | ament_python | awm_core | ROS / plain python |
| awm_description | ament_cmake | ur_description, ur_moveit_config, xacro | ROS |
| awm_bringup | ament_python | awm_description, moveit_configs_utils, controller_manager, mock_components | ROS |
| awm_path | ament_python | awm_core, awm_geometry (subprocess), rclpy, moveit_msgs | ROS |
| awm_program | ament_python | awm_core, awm_path (client), rclpy, moveit_msgs | ROS |
| awm_verify | ament_python | everything above, control_msgs | ROS |

**OCC bridge.** ROS-side code never imports OCC. `awm_core.occ_bridge.run_geometry_cli(args)` builds the
command from parameter `occ_python_cmd` (default: env `AWM_OCC_PYTHON`), sets `PYTHONPATH` to the install
locations of `awm_geometry` and `awm_core` (found with `importlib.util.find_spec` in the ROS python, so no
absolute path in source), runs `-m awm_geometry.cli ...`, and raises `GeometryCliError(stderr)` on exit ≠ 0.

## 2. Job package and files (DES-001 §5.2 + CP-3)

```
<repo_root>/
├── cell/
│   ├── wps/WPS-*.json                 D05  (SAMPLE)
│   ├── calibration/<id>.json          D08  (NOMINAL) ; active.json = copy of the active record + `source` name
│   └── io_map.yaml                    D11  TODO(OI-1) placeholder
└── jobs/<PartNo>_<PartRev>/
    ├── model.step                     D01 copy (read-only)
    ├── part.json                      D03
    ├── cache/model.brep, mesh.stl, edges.json, seam_geometry.json   derived
    ├── seams.json                     D04
    ├── process.json                   D06
    ├── path.json                      D07 (part frame)
    ├── programs/<P###>/program.json   D09 (base frame, joint segments + events)
    ├── programs/<P###>/verify/report.json, records/*.json   D15, A72/A73 records
    └── release.json                   D10
```
`active.json` is a copy (not a symlink: the repository is backed up to a Windows shared drive).

## 3. JSON schemas (draft 2020-12, in `awm_core/schemas/`)

Common header (every file): `schema_version` ("1.0"), `created_by` ("<tool> <version>"), `created_at`
(ISO 8601 UTC), `inputs`: list of `{name, sha256}` (paths relative to the job / repo root).
Readers recompute input hashes and refuse stale inputs (`StaleInputError`).
Units: SI unless the field name says `_mm`, `_deg`, `_mm_s`, `_l_min`, `_A`, `_V`, `_W`, `_m_min`.
Poses: `{"position_m": [x,y,z], "quat_xyzw": [x,y,z,w]}`.

| Schema | Key fields |
|---|---|
| part (D03) | part_no, part_rev, step_file, step_sha256, unit_source ("mm"), bbox_m {min,max}, counts {solids, faces, edges}, checks [{name, ok, detail}], healed (bool), mesh {file, sha256, linear_deflection_m, triangles}, brep_cache, edges_file |
| edges (derived) | edges: [{id "E<n>", curve_type, length_m, start_m, end_m, mid_m, faces ["F<n>",…], convexity (concave/convex/smooth/free), polyline_m [[x,y,z]…], fingerprint (sha256 of rounded type/length/end points)}] |
| seams (D04) | seams: [{seam_id "S01", edges ["E12",…], reversed (bool), joint_type (fillet/butt/lap/groove), weld_side (both faces / face id), leg_mm, sequence, process_hint (MIG/LASER/null), material, thickness_mm, start_m, end_m, length_m, fingerprint}] |
| wps (D05) | wps_id, status ("SAMPLE"), process (MIG/LASER), material, joint_types [], thickness_mm {min,max}, leg_mm {min,max}, parameters {…}, motion {travel_speed_mm_s, work_angle_deg, travel_angle_deg, stickout_mm, focus_offset_mm, weave_width_mm, weave_freq_hz}, timing {pre_flow_s, post_flow_s, crater_fill_s, arc_ok_timeout_s} |
| process (D06) | seams: [{seam_id, process, wps_id, wps_sha256, parameters, motion, timing}] |
| seam_geometry (derived) | seams: [{seam_id, spacing_m, points: [{s_m, p_m, t, n1, n2}]}] |
| path (D07) | frame "part", tool ("tcp_mig"/"tcp_laser"), calibration_id + sha256 used for the reach check, seams: [{seam_id, process, travel_speed_m_s, approach_pose, weld_poses [...], retract_pose, reach {fraction, ik_seed_joints, min_manipulability, ok}, collision {ok}}] , rejected: [{seam_id, reason}] |
| calibration (D08) | calibration_id, status ("NOMINAL"/"MEASURED"), base_T_fixture, fixture_T_part, tcp {tcp_mig, tcp_laser} (tool0_T_tcp) |
| program (D09) | program_rev, frame "base_link", calibration_sha256, joint_names, home_joints, segments: [{segment_id, kind (air/approach/weld/retract), seam_id, points [{t_s, q_rad[6]}], duration_s}], events: [{event_id, type (WeldStart/WeldEnd), seam_id, after_segment, wps_id, parameters, timing}], cycle_time_s |
| release (D10) | program_rev, program_sha256, report_sha256, scope ("simulation"/"production"), records [{type, file, sha256}], approved_by, approved_at, locked (true) |
| verify report (D15) | program_rev, program_sha256, checks {reach, collision, start_state, limits, events_at_rest} each {ok, detail}, replay {executed, segments_ok, max_tracking_error_rad, measured_time_s}, cycle_time_s, result ("PASS"/"FAIL") |
| record (A72/A73) | record_type (dry_run/trial_weld), program_rev, operator, date, result (PASS/FAIL), speed_override, notes |

## 4. ROS interfaces needed for A1–A7

Offline chain uses only MoveIt 2 / ros2_control interfaces (DES-001 §7.1 last two rows):
`/compute_ik` (GetPositionIK), `/compute_fk` (GetPositionFK), `/compute_cartesian_path` (GetCartesianPath),
`/plan_kinematic_path` (GetMotionPlan, OMPL, air moves), `/check_state_validity` (GetStateValidity),
`/apply_planning_scene` (ApplyPlanningScene), `/joint_trajectory_controller/follow_joint_trajectory`
(FollowJointTrajectory, mock hardware replay), `/joint_states`.
`awm_interfaces` gets the §7.1 runtime definitions (CellCommand.srv, CellState.msg, ExecuteSegment.action,
WeldEvent.action, PlcStatus.msg, Alarm.msg) as v0 so later runtime packages can build against them.

## 5. Algorithms (summary)

- **A11/A12** STEPControl_Reader (mm → m scaling at the boundary), ShapeFix_Shape, BRepCheck_Analyzer,
  solid/shell counts, bbox, small-edge check. **A13** BRepMesh_IncrementalMesh → binary STL in metres.
  **A14** part.json with SHA-256 of model.step.
- **A21** edge catalogue from `TopExp.MapShapes` (stable indexed map) with adjacency (`MapShapesAndAncestors`)
  and convexity from the face normals at the edge midpoint; picking by edge id or by nearest point (UI click).
  **A22** attributes validated (edges connected into one chain, concave for fillet). **A24** seams.json.
- **A41** sample each seam chain at `sample_spacing_m` with tangent t and the two face normals n1, n2.
- **A42** fillet / groove: z_tool = −normalize(n1+n2) rotated by work-angle offset; travel angle tilts z towards
  t (push > 0); x_tool = travel direction projected ⟂ z; y = z × x. Laser: TCP offset `focus_offset_mm` along z.
  Weave ≠ 0 → seam rejected ("weave planned for R3").
- **A43** approach / retract = start / end pose moved by `approach_distance_m` along −z.
- **A44/A45** part pose from the active calibration; part mesh added to the planning scene;
  IK of the approach pose with several seeds (pick_ik), then `compute_cartesian_path` approach→start,
  weld waypoints, end→retract with `avoid_collisions=true`. Accept only `fraction == 1.0` for every segment;
  otherwise the seam is rejected with the reason (no fallback).
- **A61** base_T_tcp = base_T_fixture · fixture_T_part · part_T_pose (tool frame = tcp link of D14, checked
  against D08 TCP by FK). **A62** segments: air (OMPL joint plan home→approach / retract→next approach /
  →home), approach, weld, retract (Cartesian). Time parameterisation: weld segment at constant TCP speed
  from WPS travel speed (arc length from FK, short accel ramps), others trapezoidal with config velocity /
  acceleration scaling; joint velocity limits checked. **A63** WeldStart after approach, WeldEnd after weld —
  both at segment boundaries where velocity is zero. **A64** `programs/P###/program.json`, read-only after release.
- **A71** replay all segments on mock hardware through the JTC action; check start-state, tracking error,
  state validity of every trajectory point; report D15. **A72/A73** record CLI + templates.
  **A74** release.json; scope `production` needs PASS dry-run and trial-weld records, `simulation` needs only
  the PASS report and is marked as such. Program directory made read-only.

## 6. Task checklist (ordered by dependency; each with its test)

| # | Task | Test |
|---|---|---|
| T1 | awm_core: hashing, json io + header, schemas, frames, repository paths, occ bridge | pytest (no ROS): schemas valid, round trips, stale-hash detection, frame math |
| T2 | awm_geometry: STEP import/heal/mesh/part.json, edge catalogue, test-part generator | `$AWM_OCC_PYTHON -m pytest`; colcon test runs the CLI via subprocess |
| T3 | awm_geometry: seam sampling (points, tangents, normals) | pytest on generated plate/flange parts |
| T4 | awm_cad: import CLI wrapper (A11–A14) | pytest with fixture JSON + optional real CLI |
| T5 | awm_seam: seam define/pick/export (A21, A22, A24) | pytest on fixture edges.json |
| T6 | awm_process: WPS library + assignment (A31–A33), SAMPLE WPS files | pytest |
| T7 | awm_interfaces v0 | colcon build |
| T8 | awm_description: UR5e + torch + Pismo frame + fixture xacro, SRDF, MoveIt config | xacro → URDF parse test; check_urdf-like pytest |
| T9 | awm_bringup: sim.launch.py (mock hardware, move_group, RViz/headless), D16 config, repo template | launch smoke test with timeout (move_group services up) |
| T10 | awm_path: pose generation (pure) | pytest (no ROS) |
| T11 | awm_path: MoveIt client (IK, Cartesian, scene) → path.json | launch_testing with sim headless |
| T12 | awm_program: transform + segments + time param + events (pure parts) | pytest |
| T13 | awm_program: MoveIt-backed program build → program.json | launch_testing |
| T14 | awm_verify: replay + report, records CLI, release | pytest + launch_testing |
| T15 | run_chain end-to-end on 2 generated parts | pytest e2e + `ros2 run awm_verify run_chain` |
| T16 | Docs: READMEs, USER_GUIDE, TRACEABILITY, FINAL_REPORT | — |

## 7. Reuse map (demo → new) — see `docs/generated/demo_discovery.md` for code locations

The demo is ROS 2 Jazzy C++/Python with hard-coded seams. Its logic was used as reference; no file
was copied verbatim (the demo code depends on assembly-mode data, absolute paths and a dual-robot
model). Findings fixed by the new design:

| Demo file (hold_and_weld_*) | New file | Findings fixed |
|---|---|---|
| planning/occt loader + seam_extractor_occt.py | awm_geometry/occ_io.py, topology.py, sampling.py | F03 (no hard-coded seams), F04 (pythonocc in conda env), units mm→m explicit, OCC 8 API (`topods.Edge`, `brepbndlib.Add`, no `HashCode`), arc-length sampling, edge orientation respected |
| planning/weld_planner.py (torch frame) | awm_path/poses.py | right-handed tool frame (demo z-flip bug), work / travel angle from WPS, approach / retract along the torch axis |
| application/welder_action_server.cpp (planning part) | awm_path/motion.py, planner.py | F01 (100 % Cartesian, was 35 %), F07 (collision check on every segment, no blind fallbacks, no broad ACM), F12 groundwork |
| application/welder_action_server.cpp (execution part) | awm_program/generator.py (offline), awm_verify/replay.py (sim) | ADR-05 offline/online split, F12 constant TCP speed, F14 calibration hash in program |
| application/dual_robot_coordinator.cpp | — (M8 out of scope) | F02: seam ids only from seams.json / program |
| description/robot2_welder.xacro, welding_torch_prefix.xacro, ur5e_arm_*.xacro | awm_description/urdf/*.xacro | F10 (UR5e only), F13 (mock_components instead of gz), torch TCP from config (π instead of 3.14) |
| bringup/config/moveit/*.yaml, launch/*.py | awm_description/config, awm_bringup/launch/sim.launch.py | F05 (ur_description, pick_ik declared), F06 (no absolute paths), F08 (one D16 params file) |
| calibration_tools (47 scripts) | — | not reused (DES-001 §10) |
| gripper server / gripper_sampler | — | parked (F10) |

## 8. Decisions and risks

| # | Decision / risk | Reason / fallback |
|---|---|---|
| D1 | ROS 2 Jazzy in Docker (`awm_dev`) | CLAUDE.md; change proposal CP-1 |
| D2 | Seams defined via edge catalogue (pure Python), OCC only for import and sampling | Keeps ROS side testable without conda; one geometry source |
| D3 | Edge picking by id or nearest point | Same API the Qt GUI will use for a click |
| D4 | `active.json` is a copy, not a link | Shared drive backup |
| D5 | Release scopes simulation / production | A72/A73 are not executable here; never fake records |
| D6 | Planning scene part mesh from `cache/mesh.stl` (own STL reader) | No extra deps |
| D8 | Partial seams (`range_m`, A22 start / end) | 360° orbit not reachable from a fixed base (see PROGRESS.md) |
| D9 | `joint_path` stored in path.json (D07, A44 "joint path") and reused by M6 | IK solvers are stochastic; M6 verifies by FK instead of re-solving |
| D10 | KDL for Cartesian (`weld_arm`), pick_ik global for IK search (`weld_arm_search`) | Deterministic Cartesian tracking; global branch search |
| D11 | Weld segments checked against joint limits, other moves against limits × velocity_scaling | Weld speed is a process value; no limit raised |
| D12 | Dense (0.02 rad) collision check of all segments in M6 (air moves) and A71 | move_group checks only time-parameterised waypoints |
| D7 | Fixture and Pismo frame dimensions are NOMINAL placeholders `TODO(OI-6)` style values in YAML | Real drawings not available |
| R1 | pick_ik success rate / solve time | Multiple seeds; fallback to KDL configurable in kinematics.yaml (logged, not silent) |
| R2 | compute_cartesian_path fraction < 1 near singularities | Reject seam with reason; tune part placement in test data only |
| R3 | OCC 7.x → 8.0 API differences (HashCode removed, static methods) | Use `TopTools_IndexedMapOfShape`, `topods.Edge`, `brepbndlib.Add` |
| R4 | DDS interference with the demo container | Domain 42, localhost discovery, private IPC |
