# Traceability: function → package → file → test → status

Scope of this delivery: A1, A2, A3, A4, A6, A7 (offline chain, Phase 1 R1 + offline part of R2).
Status: **Done** = implemented and tested by running; **Done (sim)** = verified on mock hardware
only; **Records only** = software side only (no robot); **Out of scope** = not in this delivery.
All tests below passed on 2026-10-19 (`colcon test`: 115 tests, 0 failures; OCC suite in conda:
12 passed).

| ID | Function | Package | Main file(s) | Tests | Status |
|---|---|---|---|---|---|
| A11 | Load STEP file | awm_geometry, awm_cad | `awm_geometry/occ_io.py` (`read_step`), `awm_cad/__init__.py` (`import_step`) | `test_geometry_occ.py::test_part_record`, `::test_step_in_metres_reads_same_size`, `::test_import_missing_step_fails`; `test_import_step.py::*`; `test_cli_subprocess.py::test_import_via_bridge` | Done |
| A12 | Check & heal model | awm_geometry | `topology.py` (`check_and_heal`) | `test_geometry_occ.py::test_part_record` (checks), `test_import_step.py::test_cli_reports_geometry_error` | Done |
| A13 | Display 3D model | awm_geometry | `occ_io.py` (`tessellate_to_stl`) → `cache/mesh.stl`; RViz via `sim.launch.py` | `test_geometry_occ.py::test_part_record` (mesh), `test_planner_sim.py` (mesh in planning scene) | Done (mesh); Qt viewer out of scope |
| A14 | Create part record | awm_geometry, awm_core | `cli.py` (`cmd_import`), `schemas/part.schema.json` | `test_geometry_occ.py::test_part_record`, `test_jsonio.py::*` | Done (basic, R1) |
| A21 | Select seam edges | awm_geometry, awm_seam | `topology.py` (`edge_catalogue`), `awm_seam/seams.py` (`EdgeCatalogue.pick`, `order_chain`) | `test_geometry_occ.py::test_edge_catalogue_plate`, `::test_edge_catalogue_pipe`; `test_seams.py::test_pick_nearest_edge`, `::test_chain_order_and_direction` | Done (CLI / API; GUI out of scope) |
| A22 | Define seam attributes | awm_seam | `seams.py` (`build_seam`, `_seam_range`) | `test_seams.py::test_build_seam`, `::test_invalid_requests`, `::test_partial_seams_on_closed_edge`, `::test_partial_seam_on_open_chain` | Done |
| A23 | Detect seams automatically | — | plug-in point: `SeamRequest` | — | Out of scope (Phase 2) |
| A24 | Export seam list | awm_seam | `seams.py` (`export_seams`, `rematch`), `schemas/seams.schema.json` | `test_seams.py::test_define_export_remove`, `::test_cli`, `::test_rematch`, `::test_stale_edges_refused` | Done |
| A31 | Select process per seam | awm_process | `__init__.py` (`select_process`) | `test_assign.py::test_select_process` | Done (MIG; LASER selectable, no laser head in D14) |
| A32 | Look up WPS | awm_process | `__init__.py` (`find_wps`, `load_library`) | `test_assign.py::test_find_wps_prefers_approved`, `::test_find_wps_reasons`, `::test_library_errors` | Done (SAMPLE WPS) |
| A33 | Assign weld parameters | awm_process | `__init__.py` (`assign`), `schemas/process.schema.json` | `test_assign.py::test_assign_writes_process_json`, `::test_cli_exit_codes` | Done |
| A41 | Discretize seam | awm_geometry, awm_path | `sampling.py` (`sample_seam`), `planner.py` (`ensure_seam_geometry`) | `test_geometry_occ.py::test_sample_straight_seam`, `::test_sample_closed_seam_and_stale_input`, `::test_sample_partial_closed_seam` | Done |
| A42 | Torch orientation & weave | awm_path | `poses.py` (`torch_axis`, `tool_pose`) | `test_poses.py::test_bisector_axis`, `::test_reference_face_work_angle`, `::test_travel_angle_push`, `::test_tool_pose_frames`, `::test_seam_poses_rejections[weave]` | Done (angles); weave R3 (rejected with reason) |
| A43 | Approach / retract moves | awm_path, awm_program | `poses.py` (`offset_along_tool_z`), `generator.py` (air moves) | `test_poses.py::test_approach_offset`, `test_generate_sim.py::test_program_structure` | Done |
| A44 | Solve IK & check reach | awm_path | `motion.py` (`ik_candidates`, `plan_seam_motion`), `moveit_client.py` | `test_planner_sim.py::test_plate_both_seams_accepted`, `::test_pipe_partial_seams_accepted`, `::test_unreachable_seam_rejected_with_reason`, `::test_tcp_mismatch_stops_planning` | Done (sim) |
| A45 | Check collisions | awm_path | `motion.py` (Cartesian with `avoid_collisions`, `first_collision`), `planner.py` (part mesh in scene) | `test_planner_sim.py::*`, `test_e2e_chain.py::test_full_orbit_pipe_seam_rejected` | Done (sim) |
| A51-A53 | Calibration | — | NOMINAL `repo_template/cell/calibration/active.json` | `test_config_and_template.py::test_nominal_calibration_matches_cell_model` | Out of scope (NOMINAL data only) |
| A61 | Transform path to robot frame | awm_core, awm_program | `calibration.py` (`base_T_part`), `generator.py` (`verify_joint_path`) | `test_calibration.py::test_load_active`, `test_generate_sim.py::test_calibration_change_makes_path_stale` | Done (nominal calibration) |
| A62 | Generate motion program | awm_program | `generator.py`, `timing.py` | `test_timing.py::*`, `test_generate_sim.py::test_program_structure`, `::test_weld_runs_at_wps_speed` | Done (ROS trajectory, ADR-01); URScript fallback R3 |
| A63 | Insert weld & I/O events | awm_program | `generator.py` (`_Builder.event`) | `test_generate_sim.py::test_program_structure`, `test_checks_records_release.py::test_static_checks_detect_problems` | Done (WeldStart / WeldEnd at rest; I/O map waits for DES-002) |
| A64 | Store & transfer program | awm_program, awm_core | `generator.py` (`programs/P###/program.json`), `repository.py` | `test_repository_and_io.py::test_job_paths`, `test_generate_sim.py::test_rejected_seam_blocks_program` | Done (store; transfer to robot = M9, out of scope) |
| A71 | Simulate program | awm_verify | `verify.py`, `replay.py` | `test_checks_records_release.py::test_static_checks_*`, `test_e2e_chain.py::test_chain_releases_program[*]` | Done (sim, mock hardware; URSim out of scope) |
| A72 | Dry run on robot | awm_verify | `records.py`, `templates/dry_run.json` | `test_checks_records_release.py::test_record_template_and_cli`, `::test_record_validation` | Records only |
| A73 | Trial weld on coupon | awm_verify | `records.py`, `templates/trial_weld.json` | `test_checks_records_release.py::test_production_needs_records` | Records only |
| A74 | Release program | awm_verify | `release.py` | `test_checks_records_release.py::test_release_simulation_and_lock`, `::test_production_needs_records`, `::test_report_of_other_program_version_refused`, `::test_changed_input_refused`; `test_e2e_chain.py::test_chain_releases_program[*]` | Done |
| A8x, A9x | Runtime, inspection | awm_interfaces (v0 only) | `msg/`, `srv/`, `action/` | build only | Out of scope |

Supporting items: D14 cell model (`awm_description`, `test_description.py`, 6 tests), D16
configuration (`awm_bringup/config/awm_params.yaml`, `test_config_and_template.py`,
`awm_path/test/test_params.py`), simulation bring-up (`test_sim_launch.py`).
