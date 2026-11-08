# G1.3 golden run — kinematic 7-axis planner (collision OFF), golden part 564082

Date 2026-11-08 · cell-sim agent (close-out C1 / G1.22) · branch `g1-22-golden-rebuild` · regenerated from
`golden_summary.json` of `scripts/golden_rebuild.py` (replaces the pre-TCP-fix run of 2026-11-06) · machine data:
[`g1-3_golden_run.json`](g1-3_golden_run.json).

**Kinematic only.** Collision is not evaluated (Goal 1, CP-58): D07 carries `collision_checked: false`,
`min_clearance_mm: null`. Cross-check 4 is red and is overridden for kinematic planning only (CP-62, record below);
every release refuses while check 4 is red.

## Set-up (everything from the cell STEP)

Only input: the cell STEP `APLSMN601_Automated Weldment System.step` (SHA-256 `bc66c09e…7e824`). All picks come from
`src/awm_bringup/test/data/golden_part.yaml`; the chain runs through an in-process `awm_api` with the real CLIs.

| Step | Result |
|---|---|
| S10 cell import, up axis +Y | 152 s; roles from the data file; `564082-Assembly-3 parts:2` and the top-level duplicate `ASSY, Welding tool:1` excluded (OI-23, decision 2026-11-08); collision scene 15 objects, no welding-tool object |
| Positioner axis | `Part1:1` edge E3, circle r 15.000 mm, axis file (1, 0, 0) |
| Head STEP | robot component cut from the cell STEP with `make-job-step`, placed in the robot's own frame (transform read from the cell component tree) |
| TCP teach (CP-61) | `LightWELD:1` edge E3955, mouth circle r 2.5 mm, stick-out 0 → `tool0_T_tcp` = (43.8288, 0.0000, 279.6056) mm, beam 60° from the flange axis; expected (43.829, 0, 279.606): off by 0.0004 mm (tolerance 0.01 mm) |
| Job STEP | `make-job-step`: Part1–3, pulleys :1 / :4, `564082-Assembly-3 parts:2`; S1 assembly import 64 s |
| Job roles / axis | rotating set `rotary_fixture`, the assembly `part`; axis E3 r 15.000 mm |
| Cross-check | 1 positioner match green (5 components, φ 0.000°), 2 / 3 skipped (no robot / tool in the job), **4 red**: Jig:1, Timing Belts:1, Timing Belts:2 in the swept volume, 5 / 6 green |
| CP-62 override | by `golden-adm` (Admin), 2026-11-08T09:29:02Z, scope `kinematic_planning`, check 4, findings Jig:1, Timing Belts:1, Timing Belts:2; reason: "G1.22 golden rebuild: check 4 is red because the bearing seats of Jig:1 and the timing belts are not separate components in the cell CAD (OI-23); kinematic-only planning, collision is Goal 2 (CP-62, decision 2026-11-08)." Carried into D07 and D09 (`cad_check_override`) |
| Seams | S01 = J1 = joint JE9 (pipe nipple ↔ solid), S02 = J2 = joint JE11 (solid ↔ elbow); butt, closed, 1 arc, 2 mm overlap, `sync`. J1 coaxial with the positioner axis (r spread 1e-9 mm), J2 not (axis 90°) |
| Plan | travel speed 2.3 mm/s (Engineer input from the data file), collision off, head `hcad` from the repository |

## Results

| Seam | Coverage | Points | θ travel (deg) | Wrist-3 travel (deg) | Label | Max travel speed (D07) | FK vs D07 |
|---|---|---|---|---|---|---|---|
| J1 `sync` | **100 %** | 88 | 368.6 | 0.0 | S+E+W+ | 6.96 mm/s | 0.0 mm |
| J2 `sync` | **100 %** | 88 | 117.8 | 159.8 | S-E+W+ | 2.39 mm/s | 0.0 mm |

No arc rejected. J1 is welded by the positioner alone (robot joints constant, θ 360° + overlap); J2 moves robot and
positioner together, wrist-3 travel 159.8° (F11 limit 180°). The planned 2.3 mm/s is below both maxima (the golden
values J2 2.39, J1 6.96 mm/s); the program step accepts it without a re-plan (see G1.5).

Golden tests on the rebuilt repository (`AWM_CELL_STEP` set): `test_golden_g13.py` 2 passed (sync J1 + J2: 2 arcs
accepted, payload reproducible; Mode C J1: 1 arc accepted, robot at rest).

## Reproduce

```
scripts/dx "python3 scripts/golden_rebuild.py --cell-step <cell STEP>"        # 14 min from scratch, then cached
AWM_CELL_STEP=<cell STEP> scripts/dx "python3 -m pytest -s src/awm_path/test/test_golden_g13.py"
```
