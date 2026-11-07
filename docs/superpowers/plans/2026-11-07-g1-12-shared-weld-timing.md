# G1.12 — One weld timing model for planner and program (2026-11-07)

Owner: program-verify. Branch `g1-12-shared-weld-timing` (from GotoCode f599d26).
Refs: DES-001 Rev 0.5 §4.3 step 5 (A47), §6.2 (events at rest), ADR-05; CP-59 follow-up of 2026-11-06 (travel speed
refused with the feasible maximum, never slowed); decision Truong 2026-11-07 (one source for the weld timing).

## Goal
`awm_program` uses `awm_path.core.weld_timing` for the weld spline, its maxima and the maximum feasible travel speed,
so the planner (D07) and the program (D09) cannot disagree.

## Tasks (TDD)
1. Tests first (`src/awm_program/test/test_core_shared_timing.py`, `test_core_program7.py`):
   - the program's maximum equals the planner's maximum on the seeded non-monotonic paths of
     `awm_path/test/test_core_weld_timing.py` (seed 8, seed 178 with a feasible request);
   - the program helpers are the planner functions (no copy);
   - D07 `planner.params.sync` ramp / resolution different from `ProgramParams` → refused; missing → refused
     with "re-plan".
2. `core/spline.py`, `timing.py`: `TimingError`, `constant_speed_times`, `clamped_cubic`, `max_abs`, `weld_times`
   taken from `awm_path.core.weld_timing` (thin wrapper only where the signature differs).
3. `core/program7.py`: weld spline, feasibility and `max_feasible_speed` from `weld_timing`; refuse a D07 whose ramp /
   resolution differ from `ProgramParams` or are missing; refusal text no longer says "velocity only".
4. `description.py`: acceleration limits from `PlanningDescription.sync_cell` (same file, same conversion, already in
   the D07 description record) instead of a second read; `ProgramCell` refuses acceleration limits that differ from
   its `SyncCell`.

## Decision (item 2 of the brief)
Refuse on mismatch rather than silently adopting the D07 values: `ProgramParams` is recorded in every D09 as the
generator's parameters, and the weld ramp is the motion the program commands. A ramp different from the one the
planner checked makes the D07 speed verdict invalid, so the honest answer is "re-plan", not a quiet override.

## Test
`scripts/dx "colcon build --symlink-install --packages-select awm_core awm_path awm_program && colcon test
--packages-select awm_program && colcon test-result --verbose"`; golden `test_golden_g15.py` and
`awm_path/test/test_golden_g13.py` with `AWM_SPIKE_REPO`, `AWM_OCC_PYTHON` (J2 2.39, J1 6.96 mm/s expected).
