# G1.5 — synchronised 7-joint program D09 from the G1.3 path D07 — branch plan

> Branch `g1-5-program-7axis` (agent `program-verify`, worktree `.worktrees/program-verify`, ROS_DOMAIN_ID 47).
> Master plan: `2026-11-05-goal1-coverage-ui-to-sim.md` row G1.5, Global Constraints, Review Focus 5.
> Spec: CP-58 (Goal 1, collision not evaluated, `collision_checked: false`, every release gate refuses), CP-59
> (synchronised 7 axes, follow-up decisions 2026-11-06: travel speed is an Engineer input, never slowed silently, no
> limit raised; Mode C FK tolerance F8 0.05 mm / 0.1°, sync 0.01 mm / 0.01°), DES-001 Rev 0.5 §4.3 steps 5–6, §5.3
> (frame chain with θ, D08 hash into D09), §6.2 (WeldStart / WeldEnd only with robot **and** positioner at rest),
> ADR-05, ADR-17; PLAN.md T2.5 (Goal 1 subset).

## Scope

1. **D09 v2 MINOR 2.1** (`awm_core/schemas/v2/program.schema.json`, example): `controller: robot_positioner`
   (one 7-joint trajectory, order `joint_names.robot` then `joint_names.positioner`, `hold_deg: []`), program-level
   `collision_checked` (required from 2.1), `positioner_mode` (`sync | C | B`) required on robot_positioner weld
   segments, optional segment `collision_checked`, optional `generator` record; 2.0 documents stay valid.
2. **Pure generator core** `awm_program/core/` (no rclpy): rest-to-rest and weld timing, C2 cubic spline velocities /
   accelerations, approach / retract IK on the weld branch, checks (FK, events at rest, TCP speed relative to the
   part, limits). **CLI** `python3 -m awm_program.cli generate --job <dir> [--path <file>] [--head-dir <dir>]` with
   the `progress` / `result` line protocol of `awm_path.cli`.
3. **Release gate** `awm_verify.gate`: a program with `collision_checked: false` (or without it, or with an event
   without a process assignment) is refused for every scope; `awm_verify.release.release` calls it first.

## Design decisions (to confirm in review)

- **Description values = one source.** The CLI renders the description exactly as the planner did
  (`awm_path.description.load_planning_description` with the D07 head model) and refuses if its record differs from
  `planner.params.description` in D07. Joint accelerations from `awm_description/config/joint_limits_v2.yaml` and
  `positioner_limits.yaml`; home = `initial_positions_v2.yaml` (mock-hardware start state, so G1.7 start-state check
  holds). Tolerances (FK, |Δq| max, singularity, wrist 3) from the D07 `planner.params.sync` record.
- **Calibration.** Goal 1 D07 is planned with the nominal frames; the generator recomputes the NOMINAL record from
  `frames.yaml`, `positioner_limits.yaml`, UR kinematics and the head file and refuses when it differs from the D07
  context (stale). If `cell/calibration/active.json` exists the generator refuses: the 7-axis chain needs a D08 v2
  with the positioner axis, which does not exist yet (stop item for the lead).
- **Weld timing (A47).** Times along the D07 s values with a trapezoidal speed profile: rest → WPS travel speed →
  rest, ramp acceleration `weld_ramp_accel_mm_s2` (Assumption 5 mm/s², recorded). The ramp lies on the seam after
  WeldStart / before WeldEnd (events stay at rest); the constant-speed part runs exactly at the travel speed.
- **Velocities / accelerations** of every segment are those of one clamped C2 cubic spline through the points (v = 0
  at both ends). A quintic interpolation from (p, v, a) — the joint trajectory controller — reproduces this cubic, so
  the limits are checked exactly (acceleration at the knots, velocity at knots and interior extrema). Air / approach
  / retract are stretched in time until they are inside the scaled limits (scaling 0.25, Assumption); a weld segment
  is never stretched: if it breaks a velocity or acceleration limit the program is refused with the largest feasible
  travel speed (velocity from D07, velocity + acceleration by bisection).
- **Segments per arc:** air (joint-interpolated, ≤ 0.02 rad steps, `collision_checked: false`, kind `index` when only
  θ moves) → approach 40 mm along the seam +Z (Cartesian line, weld IK branch, |Δq| ≤ 5°) → WeldStart → weld
  (`weld`, Mode C `positioner_weld`, `positioner_mode`) → WeldEnd → retract 40 mm → … → air to home.
- **Events** carry `wps_id: null`, `preset: null` while no D06 v2 exists (no fabricated process data); the release gate
  refuses such a program. Needs a schema MINOR change (nullable) — reported to the lead.

## TDD steps

1. `awm_core` test: a 2.1 program with robot_positioner segments validates; 7 values required; `positioner_mode`
   required on weld segments; `collision_checked` required from 2.1; a 2.0 document stays valid → schema change.
2. `awm_program` spline / timing tests (pure): clamped spline interpolates, v = 0 at ends, exact max |v| / |a|
   against dense sampling; weld times = trapezoid with constant part at v; rest-to-rest timing within limits.
3. Generator tests on the synthetic ring job (`awm_path/test/g13_job.py`, hbox head): program validates (2.1), every
   segment 7 joints, continuity, starts / ends at home; TCP speed relative to the part within 5 % on the
   constant-speed part (θ moving); events at rest (robot and positioner velocity 0); FK of each weld point = D07 pose
   ≤ 0.01 mm / 0.01° (sync), F8 (Mode C); approach / retract 40 mm along the beam, one IK label, |Δq| ≤ 5°.
4. Refusal tests: stale D07, rejected arc, `speed_within_limits: false` (message has arc and number), acceleration
   over limit (message has the number), active D08 present; CLI exit codes and `result` lines.
5. Release gate test (`awm_verify`): `collision_checked: false` refused for `simulation`, `trial`, `production`.
6. Golden run (env `AWM_SPIKE_REPO`, skipped otherwise): plan J1 + J2 at 3.0 mm/s, generate, record results in
   `docs/generated/goal1/g1-5_golden_program.md`.
7. Full check: `scripts/dx "colcon build --symlink-install && colcon test --packages-select awm_core awm_program
   awm_verify awm_path && colcon test-result --verbose"`.
