# G1.5 golden run — synchronised 7-joint program D09 2.1 (collision OFF), golden part 564082

Date 2026-11-08 · cell-sim agent (close-out C1 / G1.22) · branch `g1-22-golden-rebuild` · regenerated from
`golden_summary.json` of `scripts/golden_rebuild.py` (replaces the pre-TCP-fix run of 2026-11-06) · machine data:
[`g1-5_golden_program.json`](g1-5_golden_program.json).

**Kinematic only.** The program carries `collision_checked: false`; air moves are joint-interpolated, not planned
around obstacles. It also carries the CP-62 override reference (`cad_check_override`: by `golden-adm`,
2026-11-08T09:29:02Z, check 4, `cad_check.json` SHA-256 `6067a2fc…bccbd6`, reason as in G1.3); the release gate
refuses this program for every scope while check 4 is red.

## Set-up

Plan of [G1.3](g1-3_golden_run.md) (J1 = JE9, J2 = JE11, both `sync`, 2.3 mm/s), then `POST /jobs/564082_A/program`
(awm_program `generate`) in the same in-process `awm_api`. Limits (never raised): UR joints 180 deg/s,
286.5 deg/s²; positioner 30 deg/s, 50 deg/s². Approach / retract 40 mm, weld ramp 5 mm/s², air moves at 25 % of
the limits (generator defaults, Assumptions).

## Results — program P001, accepted at 2.3 mm/s

| Segment | Kind | Seam | Points | Duration (s) |
|---|---|---|---|---|
| G001 | air | — | 241 | 17.50 |
| G002 | approach | J1 | 41 | 0.68 |
| G003 | weld | J1 | 88 | 37.76 |
| G004 | retract | J1 | 41 | 0.68 |
| G005 | air | — | 203 | 28.09 |
| G006 | approach | J2 | 41 | 0.82 |
| G007 | weld | J2 | 88 | 37.76 |
| G008 | retract | J2 | 41 | 0.83 |
| G009 | air | — | 203 | 6.72 |

**Cycle time 130.8 s** (motion only, no process dwell times). 4 weld events, each at a boundary with all 7 velocities
0.

| Weld | Speed | Duration | FK vs D07 | Max joint speed (deg/s) | Max joint acceleration (deg/s²) |
|---|---|---|---|---|---|
| J1 (G003) | 2.3 mm/s | 37.76 s | 5e-9 mm / 1e-7° | positioner 9.91 (robot 0) | positioner 28.9 |
| J2 (G007) | 2.3 mm/s | 37.76 s | 8e-9 mm / 9e-8° | ≤ 12.73 (all 7 joints) | ≤ 46.8 (positioner 46.0 of 50) |

Weld ramp 0.53 mm at each end.

Other checks on the rebuilt repository (`AWM_CELL_STEP` set):

- `test_golden_g15.py` 2 passed: at the test's requested 3.0 mm/s the generator refuses J2 and names
  `max_travel_speed_mm_s = 2.39`; re-planned at 2.39 mm/s: cycle 128.1 s (sync + sync and J1 `C` + J2 `sync`), TCP
  speed and FK checks pass.
- `test_sim_mock_7axis_launch.py::test_replay_golden` (sim_mock `repo:=<golden repo>`, 7-joint controller, mock
  hardware): P001 replayed, final deviation 0.0°, max tracking error 0.437°.
- `test_sim_mock_repo_launch.py::test_sim_mock_repo`: 15 scene objects, head H-CAD, synthetic replay ok.
- web FK check (`fk.test.ts` with `AWM_S4_GOLDEN_DIR=build/golden_repo/s4_golden`): 176 points, max 1.28e-7 mm /
  2.41e-6°.

## Reproduce

```
scripts/dx "python3 scripts/golden_rebuild.py --cell-step <cell STEP>"
AWM_CELL_STEP=<cell STEP> scripts/dx "python3 -m pytest -s src/awm_program/test/test_golden_g15.py"
```

## Open points

1. Events without process data: `wps_id` / `preset` are `null` (no D06 v2 yet); every release gate refuses it.
2. Weld ramp on the seam (0.53 mm at 2.3 mm/s) as in the 2026-11-06 run.
