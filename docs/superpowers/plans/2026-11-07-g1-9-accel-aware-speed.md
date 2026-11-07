# G1.9 — Acceleration-aware maximum travel speed in the planner (D07)

Date: 2026-11-07 · Agent: planner · Branch: `g1-9-accel-speed` (from GotoCode 4df0f25)
References: DES-001 Rev 0.5 §4.3 step 5 (A47 time parameterisation, owned by M4), §6.2 (segments start / end at
rest), ADR-14 (units), CP-59 follow-up of 2026-11-06 (travel speed = Engineer input; refuse with the feasible maximum
shown; never slow silently; never raise limits), PROGRESS.md Goal 1 next step (4).

## Problem

`awm_path.core.sync.timing` reports `max_travel_speed_mm_s` / `speed_within_limits` from joint and positioner
**velocity** limits only (finite differences at constant speed). `awm_program` times the weld as a clamped C2 cubic
spline over a trapezoidal arc-length profile (ramp 5 mm/s², Assumption) and checks **velocity and acceleration**
limits. Golden J2: D07 says 3.21 mm/s, the program accepts only ≤ 2.39 mm/s → plan "ok", program refused.

## Decision (inside `src/awm_path` only)

* `awm_program` imports `awm_path` (not the reverse), so the planner cannot import `awm_program.core.spline`.
  A47 is an M4 function: the weld timing model (trapezoidal arc-length profile + clamped cubic + exact spline maxima)
  goes into **`awm_path/core/weld_timing.py`**, a faithful copy of the `awm_program` model. Request to program-verify:
  replace the copy in `awm_program.core.spline` / `timing` by imports from `awm_path.core.weld_timing` (one source)
  and read the ramp acceleration from D07 `planner.params.sync`.
* Limits: same sources as `awm_program.description` — UR accelerations from
  `awm_description/config/joint_limits_v2.yaml` (rad/s² → deg/s² through `awm_core.units`), positioner from
  `positioner_limits.yaml` (`awm_bringup.cell_config`). Carried in `SyncCell` (new required fields
  `joint_acceleration_deg_s2`, `theta_acceleration_deg_s2`; no velocity-only fallback).
* `SyncParams` gains `weld_ramp_accel_mm_s2 = 5.0` and `speed_search_resolution_mm_s = 0.01` (Assumptions, same
  values as `ProgramParams`), recorded in D07 `planner.params.sync`.
* `timing()` keeps the D07 contract keys (schema `planning` has `additionalProperties: false`):
  `speed_within_limits` = the spline at the requested speed is inside velocity **and** acceleration limits;
  `max_travel_speed_mm_s` = largest speed (bisection on (0, √(a_ramp·L)), rounded down to the resolution) inside
  both; `max_joint_speed_deg_s` = exact spline velocity maxima at the requested speed. D07 `timed` times unchanged
  (t = Δs / v; the program re-times). Schema description texts ("velocity limit") → request to the lead.

## Tasks

- [x] T1 Plan (this file).
- [x] T2 Tests first (`src/awm_path/test/test_core_weld_timing.py`): synthetic 7-joint path where the velocity-only
  maximum is feasible for velocity but breaks an acceleration limit → `speed_within_limits` false and the reported
  maximum < velocity-only maximum; the reported maximum is accepted by the same model and maximum + 0.02 mm/s is
  refused; parity with `awm_program.core.program7.max_feasible_speed` / `_within` (within 0.01 mm/s, the search
  resolution); missing acceleration limits refused; `SyncCell` / description carry the accelerations.
- [x] T3 Implement `weld_timing.py`, `SyncCell` fields, `SyncParams` fields, `timing()`, description loader.
- [x] T4 Update the existing synthetic `SyncCell` builders in `awm_path` tests.
- [x] T5 Golden (if `AWM_SPIKE_REPO`): J1 / J2 max speed planner vs program.
- [x] T6 `scripts/dx "colcon build --symlink-install --packages-select awm_core awm_path && colcon test
  --packages-select awm_path && colcon test-result --verbose"`; report.

## Result (2026-11-07)

Golden (spike repo, H-CAD, collision off, requested 3.0 mm/s): J2 `sync` planner max 2.39 mm/s (was 3.21,
velocity only) = program max 2.39; J1 `sync` and J1 `C` planner 6.96, program bisection 6.95 — the program accepts
6.96 and refuses 6.98 (difference = the program's bisection floor, within the 0.01 mm/s resolution).

T6: `colcon test --packages-select awm_path` → awm_path 247 tests, 0 errors, 0 failures, 2 skipped
(`AWM_SPIKE_REPO not set`); `colcon test-result` summary 398 / 0 / 0 / 3 includes older result files of awm_core,
awm_bringup, awm_seam (1 skip `AWM_SPIKE_REPO not set`) and awm_description in this worktree's build folder.
With `AWM_SPIKE_REPO`: golden g13 + g15 4 passed.

Review minors (2026-11-07): feasibility is not always monotonic in the travel speed (reviewer: 2 of 300 random
paths) → `max_feasible_speed` scans 256 speeds before the bisection and never reports a feasible request above the
maximum; remaining case (island narrower than the scan step) named in the `weld_timing` docstring. The D07 speed
record is timed on the rounded values written to D07.
