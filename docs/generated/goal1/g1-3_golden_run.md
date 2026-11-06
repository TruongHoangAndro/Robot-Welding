# G1.3 golden run — kinematic 7-axis planner (collision OFF), golden part 564082

Date 2026-11-06 · planner agent · branch `g1-3-sync-planner`, planner code at commit `2e3537e` (clean checkout) ·
machine data: [`g1-3_golden_run.json`](g1-3_golden_run.json).

**Kinematic only.** Collision is not evaluated (Goal 1, CP-58): every D07 carries `collision_checked: false`,
`min_clearance_mm: null`, `scene_backend: none …`. No release gate may accept these paths (DES-001 rule 4).

## Set-up

- Job: copy of the spike job `564082_A` with `cell/model/frames.yaml` and the H-CAD head folder `cell/model/head`
  (the spike repo itself was not written). Seams built with `python3 -m awm_seam.joint_cli build` (G1.2):
  S01 = J1 = joint edge JE2, S02 = J2 = joint edge JE4; butt, closed, 1 arc, 2 mm closing overlap, pitch ≤ 1 mm.
  The coaxial test of G1.2 marks J1 coaxial (r 13.333–13.337 mm), J2 not (r 76.96–78.11 mm, axis ⟂).
- Planner: `python3 -m awm_path.cli plan --job … --collision off --out …` with the defaults head model `hcad`
  (description rendered from `awm_cell_v2.urdf.xacro` with the `frames.yaml` pose arguments), ψ grid 10°,
  θ lattice 5°, 8 IK labels, |Δq| ≤ 5° on 7 joints, F6 margins, F11 wrist 3, CP-59 cost weights (Assumption
  defaults, recorded in `planner.params`), travel speed 10 mm/s (**Assumption**, D06 v2 has no per-seam speed yet).
- Mode C: second copy with S01 = J1 in `C` (J2 not coaxial, so Mode C does not apply to it).

## Results

| Seam / mode | Result | Points / pitch | θ range (deg) | ψ | Label | max \|Δq\| 7 joints (deg) | FK vs D07 pose | Run time |
|---|---|---|---|---|---|---|---|---|
| J1 `sync` | **100 %**, accepted | 88 / 0.997 mm (1 interpolated end point) | −249.0 … +119.6 (travel 368.6) | 20° fixed | S+E+W+ | robot ≤ 0.0007, θ 4.29 | 8e-9 mm / 9e-8° | 8.4 s alone |
| J2 `sync` | **100 %**, accepted after one CP-53 refinement (pitch 1.0 mm: 1 `step` point → 0.5 mm) | 174 / 0.499 mm | −156.6 … −89.8 (travel 129.2) | 110°–320° | S+E+W+ | 4.66 (θ), 4.22 (wrist 3) | 8e-9 mm / 9e-8° | 18.2 s alone |
| J1 `C` | **100 %**, accepted | 88 / 0.997 mm | −249.0 … +119.6 | 0° | S+E+W+ | robot 0 (one state), θ 4.29 | 0.0039 mm / 0.0145° (F8 tolerance 0.05 mm / 0.1°) | 0.3 s |

Both seams in one `sync` run: 25.5 s and 25.8 s (whole CLI incl. xacro rendering; analytic IK only).
Two runs per case gave identical payloads (everything except `created_at` and `run_id`): **reproducible**.
No arc rejected, so no failing s ranges exist for this part.

Observations (reported, not acceptance rules in G1.3):

1. **J1 sync ≈ Mode C.** The optimum keeps the robot still (robot joints vary < 0.001°) and turns the positioner
   368.6° (360° + 2 mm overlap); the weld point stays at the top (seam Z vs world +Z ≤ 0.01°).
2. **J2 sync** moves robot and positioner together; the positioner only covers 67° of the range, the robot does the
   rest. Weld position is not held: seam Z up to 129.8° from world +Z (weight `w_up` = 0.05, Assumption). Wrist-3
   travel 122° (F11 limit 180°). Closure: the state at s = L differs from s = 0 by up to 116° (wrist 3) and θ 35°;
   the overlap is planned as explicit layers, F3 closure is reported, not enforced (branch plan).
3. **Joint speed at 10 mm/s.** J1 (sync and C) needs a positioner speed of 43.0 deg/s > its limit; largest travel
   speed within the limits 6.98 mm/s. J2 sync exceeds several joint limits at 10 mm/s; largest travel speed within
   the limits 3.21 mm/s. Limits are never raised; the travel speed is a D06 value — G1.5 / D06 must handle it.
4. **Mode C FK residual 0.0145°** (> 0.01°, < F8 0.1°): with the robot at rest the pose can only follow a rigid
   rotation; the B-Rep circle is not exactly coaxial (r spread 0.004 mm). The 0.01 mm / 0.01° FK check holds for
   `sync`; Mode C is held to the F8 tolerance.

## Reproduce

```
AWM_SPIKE_REPO=<spike repo root> AWM_OCC_PYTHON=<conda OCC python> \
  scripts/dx "python3 -m pytest -s src/awm_path/test/test_golden_g13.py"
```
