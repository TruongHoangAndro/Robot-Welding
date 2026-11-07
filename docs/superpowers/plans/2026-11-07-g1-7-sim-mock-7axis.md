# G1.7 — one 7-joint trajectory controller in `sim_mock` + D09 replay CLI — branch plan

> Branch `g1-7-sim-mock-7axis` (agent `cell-sim`, worktree `.worktrees/cell-sim`, ROS_DOMAIN_ID 45).
> Master plan: `2026-11-05-goal1-coverage-ui-to-sim.md` row G1.7. Design: DES-001 Rev 0.5 §6.3 Option A1 (one
> controller manager, one joint trajectory controller for the 6 UR5e joints + `positioner_joint`), ADR-11, ADR-13
> (mock hardware only), §5.1 D09 (`program.schema` v2 MINOR 2.1, `controller: robot_positioner`, from
> `awm_program.cli generate`, G1.5), §7.1. Rules: CLAUDE.md (units through `awm_core.units`, no part data in source,
> no raised limits).

## Scope

1. **Controller.** `awm_description/config/ros2_controllers_v2.yaml` declares a third controller
   `robot_positioner_trajectory_controller` (JointTrajectoryController, 7 joints: robot joints then
   `positioner_joint`, position command, same constraints as the two existing controllers, no limit changed).
   `sim_mock.launch.py` gets `trajectory_controllers:=split|robot_positioner` (default `split` = unchanged 6 + 1
   path). `robot_positioner` spawns `joint_state_broadcaster` + the 7-joint controller only (the two split
   controllers claim the same command interfaces and cannot be active with it) and points MoveIt to
   `moveit_controllers_v2_7axis.yaml`.
2. **Replay core** `awm_bringup/replay.py` (pure Python, no rclpy, TDD): load + schema-check a D09 2.x program,
   accept only `robot_positioner` segments, joint order from `joint_names`, deg → rad through `awm_core.units`,
   segment continuity, start-state check against a tolerance from config, final-state and tracking-error summary.
3. **Replay CLI** `ros2 run awm_bringup replay_program --program <program.json> [--config <yaml>] [--segments a-b]`:
   per segment: read `/joint_states`, refuse (exit 2, nothing sent) if any joint differs from the segment start by
   more than `start_state_tolerance_deg`; send the segment as one `FollowJointTrajectory` goal to the 7-joint
   controller; log max tracking error (action feedback) and the end deviation; JSON summary line `replay {...}`.
   Exit 0 ok / 2 refused / 1 error. Config: `awm_bringup/config/program_replay.yaml` (mm / deg / s, D16).
   No time scaling (the program timing is replayed as is; nothing faster than planned).
4. **Tests.** Unit tests of the core (synthetic programs = test data). Launch test `test_sim_mock_7axis_launch.py`:
   (a) always: 7-joint controller active and split controllers absent; synthetic 2-segment program replayed, final
   `/joint_states` = last point; a program not starting at the current state is refused with exit 2 and no motion;
   (b) golden: `AWM_GOLDEN_REPO` (old-flow repo, e.g. `.worktrees/demo/golden_repo_g1`), `AWM_GOLDEN_JOB`
   (default `564082_A`), latest `programs/P###` (or `AWM_GOLDEN_PROGRAM_REV`): J1 + J2 program replayed
   synchronised, final `/joint_states` = last D09 point, tracking error logged; skipped with the reason when unset.
   Existing `test_sim_mock_launch.py` (split path) unchanged and must still pass.

## Assumptions

- `start_state_tolerance_deg` 0.05 (≈ 0.001 rad of the v1 M7 value), `final_tolerance_deg` 0.05: Assumption.
- `allow_partial_joints_goal: true` on the 7-joint controller so MoveIt arm-only plans hold the positioner.

## Out of scope

Collision checking of the program (Goal 2), the `/simulation` "run in RViz" button (web), URSim (Sim-2).

## Done when

`scripts/dx "colcon build --symlink-install --packages-select awm_description awm_bringup && colcon test
--packages-select awm_description awm_bringup && colcon test-result --verbose"` green, and the golden test run once
with `AWM_GOLDEN_REPO` set (result quoted in the report).
