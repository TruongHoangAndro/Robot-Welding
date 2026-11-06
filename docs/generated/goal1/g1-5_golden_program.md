# G1.5 golden run — synchronised 7-joint program D09 2.1 (collision OFF), golden part 564082

Date 2026-11-06 · program-verify agent · branch `g1-5-program-7axis`, code at commit `f6d9fa5` ·
machine data: [`g1-5_golden_program.json`](g1-5_golden_program.json).

**Kinematic only.** Collision is not evaluated (Goal 1, CP-58): the program carries `collision_checked: false` (and
every segment too); air moves are joint-interpolated, not planned around obstacles. The release gate of
`awm_verify` refuses such a program for every scope (simulation, trial, production).

## Set-up

- Job: temporary copy of the spike job `564082_A` built by the G1.3 golden helper (`awm_path/test/test_golden_g13.py`):
  S01 = J1 (JE2), S02 = J2 (JE4), butt, closed, 1 arc, 2 mm closing overlap, pitch ≤ 1 mm; head `hcad`.
- Plan: `python3 -m awm_path.cli plan --job … --collision off --travel-speed-mm-s 3.0` (lead brief), then
  `python3 -m awm_program.cli generate --job …`.
- Generator parameters (Assumptions, recorded in `generator.params`): approach / retract 40 mm along the seam +Z,
  weld speed ramp 5 mm/s² on the seam (after WeldStart, before WeldEnd), air / approach / retract at 25 % of the
  joint velocity and acceleration limits. Limits (never raised): UR joints 180 deg/s, 286.5 deg/s²
  (`joint_limits_v2.yaml`); positioner 30 deg/s, 50 deg/s² (`positioner_limits.yaml`). Home =
  `initial_positions_v2.yaml`. Calibration NOMINAL (frames.yaml), same hash as D07.

## Results

**At the requested 3.0 mm/s the program is refused** (exit 1, nothing written):

> S02 arc 1/1: at travel speed 3.0 mm/s the weld motion breaks wrist_3_joint acceleration 412.8 > 286.5 deg/s²,
> positioner_joint velocity 33.5 > 30.0 deg/s, positioner_joint acceleration 313.4 > 50.0 deg/s²; max feasible travel
> speed 1.19 mm/s with velocity and acceleration limits (D07 max_travel_speed_mm_s 3.21, velocity only)

J2's D07 joint path has kinks: in a few steps the positioner reverses (e.g. Δθ +0.35° → −4.65° per 0.5 mm step at
s ≈ 16.5 mm and back at s ≈ 20 mm, wrist 3 likewise near s ≈ 68 mm). Every step is ≤ 5° (F1), but the velocity
jump in one step needs an acceleration that grows with v²; the velocity-only maximum of D07 (3.21 mm/s) does not
see it. J1 alone at 3.0 mm/s is accepted (P002 of the scratch run: weld 29.2 s, positioner 13.3 deg/s,
26.8 deg/s², cycle 81.6 s).

**Re-planned at 1.19 mm/s** (the reported maximum; an Engineer decision in the real flow):

| Run | Seam / mode | Weld points | Weld time | TCP speed vs part (constant part) | FK vs D07 pose | θ travel | max v / limit | max a / limit |
|---|---|---|---|---|---|---|---|---|
| sync + sync | J1 `sync` | 88 | 72.33 s | ≤ 0.023 % off 1.19 mm/s | 8e-9 mm / 9e-8° | 368.6° | 0.21 | 0.51 |
| sync + sync | J2 `sync` | 174 | 72.33 s | ≤ 0.006 % | 8e-9 mm / 9e-8° | 66.8° | 0.44 | 0.99 (wrist 3) |
| C + sync | J1 `C` | 88 | 72.33 s | ≤ 0.026 % | 0.0039 mm / 0.0145° (F8 0.05 / 0.1) | 368.6° | 0.21 | 0.51 |

Segments (both runs): air 16.8 s → approach J1 0.6 s → weld J1 72.3 s → retract 0.6 s → air 16.7 s → approach J2
1.0 s → weld J2 72.3 s → retract 0.6 s → air home 13.5 s. **Cycle time 194.4 s** (motion only; no process dwell
times, no D06). Weld ramp 0.14 mm at each end (1.19 mm/s, 5 mm/s²). WeldStart / WeldEnd: 4 events, each between two
segments whose boundary points have all 7 velocities = 0. Every segment inside its limits (weld: 100 %, others:
25 %), checked on the commanded spline (exact maxima, not only at the points).

## Reproduce

```
AWM_SPIKE_REPO=<spike repo root> AWM_OCC_PYTHON=<conda OCC python> \
  scripts/dx "python3 -m pytest -s src/awm_program/test/test_golden_g15.py"
```
(`AWM_G15_SPEED` changes the requested speed; default 3.0.)

## Open points for Truong / lead

1. **J2 at 3 mm/s is not feasible within the acceleration limits** of the description (positioner 50 deg/s²,
   Assumption OI-14; UR 5 rad/s²). Options: accept ≤ 1.19 mm/s for J2, a smoothness term in the G1.3 cost (second
   difference of θ / q6) so the planner avoids velocity reversals, or a planner-side maximum speed that includes
   accelerations. The generator never slows down and never raises a limit; it refuses and shows the number.
2. **Weld ramp on the seam** (0.14 mm at 1.19 mm/s, 0.9 mm at 3 mm/s): the first and last mm see a lower speed
   with emission on (more heat input). Events must stay at rest (§6.2), so the ramp cannot be moved off the seam
   without a run-up before the start point. Assumption parameter `weld_ramp_accel_mm_s2` = 5.
3. **Events without process data**: `wps_id` / `preset` are `null` (no D06 v2 yet); schema 2.1 allows it and every
   release gate refuses it.
