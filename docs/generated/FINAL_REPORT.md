# APLSMN601 offline engineering chain (A1–A7): final report

Date: 2026-10-19 · Workspace: `~/ros2/aplsmn601_ws` (git branch `master`, not pushed) ·
Environment: dev container `awm_dev` (ROS 2 Jazzy, image of the demo) + conda env `awm_occ`
(pythonocc-core 8.0.1).

## 1. Result in one paragraph

The offline chain STEP → part.json → seams.json → process.json → path.json → program.json →
verification report → release.json runs end to end in simulation (ros2_control mock hardware +
MoveIt 2), in one command (`ros2 run awm_verify run_chain`) and in one pytest
(`awm_verify/test/test_e2e_chain.py`). Two generated parts pass and are released with scope
*simulation*: a plate T-joint (2 fillet seams) and a pipe-to-flange joint (one circular fillet
joint welded as 3 arcs). A full 360° pipe seam is rejected at A4 with a reason, as designed.
No production release was made: dry run (A72) and trial weld (A73) need the real cell.

## 2. What was built

| Package | Module | Content |
|---|---|---|
| awm_core | M0 | 12 JSON schemas (D03–D10, D15, records, derived), validated JSON I/O with input SHA-256 and stale detection, frame math, calibration chain, repository API, OCC bridge, STL reader |
| awm_geometry | M1/M2 helper | OCCT 8 STEP import (explicit mm→m), check & heal, mesh, edge catalogue with convexity and fingerprints, arc-length seam sampling with face normals, generated test parts |
| awm_cad | M1 | `import_step` CLI / API (A11–A14) |
| awm_seam | M2 | edge picking by id or point, chain validation, attributes, partial seams (A22 start / end), export, re-match after a STEP revision |
| awm_process | M3 | WPS library (3 SAMPLE WPS), process selection, WPS lookup, process.json |
| awm_interfaces | — | DES-001 §7.1 runtime interfaces, v0 (not used by the offline chain) |
| awm_description | D14 | UR5e + MIG torch + Pismo frame + fixture, mock hardware, SRDF, MoveIt config |
| awm_bringup | — | `sim.launch.py` (headless option), D16 `awm_params.yaml`, repository template |
| awm_path | M4 | torch orientation, approach / retract, IK branch search, 100 % collision-checked Cartesian paths, path.json with joint path |
| awm_program | M6 | base-frame program, constant-TCP-speed weld timing, air moves (OMPL + dense check), WeldStart / WeldEnd at rest, revisions |
| awm_verify | M7 | static checks, dense collision check, replay on mock hardware, report, A72/A73 record CLI + templates, release + read-only lock, `run_chain` |

Environment (Phase −1): `scripts/create_dev_container.sh`, `scripts/dx`, `docker/Dockerfile`,
`docker/README.md`, `docs/generated/env_report.md`, `docs/generated/DEPENDENCIES.md`.

## 3. Test results (run, not assumed)

| Suite | Result |
|---|---|
| `colcon test` (11 packages, run 2 times in a row after the last fix) | 115 tests, 0 errors, 0 failures, 9 skipped (the 9 are the direct-OCC tests; the same tests run inside `test_occ_suite_in_conda`) |
| OCC suite in conda (`$AWM_OCC_PYTHON -m pytest src/awm_geometry/test`) | 12 passed, 1 skipped (the conda wrapper itself) |
| awm_core in conda | 23 passed |
| Path planning robustness (repeated `plan_path`) | pick_ik for Cartesian: plate 3/3, pipe 3 arcs 4/5; KDL for Cartesian: plate 5/5, pipe 5/5 |

End-to-end (`run_chain`, mock hardware, replay at 1× speed):

| Part | Seams | Program | Checks | Cycle time |
|---|---|---|---|---|
| TEST-PLATE-T rev A | S01, S02 (160 mm each, WPS-MIG-S235-FW-01, 6 mm/s) | P001: 9 segments, 4 events | PASS; 599 states collision-free; 9/9 segments replayed | 74–80 s (OMPL air moves vary) |
| TEST-PIPE-FL rev A | S01 78.5 mm, S02 52.4 mm, S03 26.2 mm (WPS-MIG-S235-FW-02, 8 mm/s) | P001: 13 segments, 6 events | PASS; 1518 states collision-free; 13/13 replayed | 53–58 s |
| TEST-PIPE-FL, one 360° seam | S01 | none | rejected at A4: no IK branch gives a 100 % collision-free path | — |

## 4. Design rules: how they are enforced

| Rule | Where |
|---|---|
| Offline / online split | programs are generated and verified offline; replay only of stored segments |
| Data-driven | no seam id, pose or part data in source; test parts carry their own pick files |
| Path in part frame | path.json `frame: part`; base frame only in awm_program with the active calibration |
| 100 % or reject | `FULL = 1 - 1e-9` in awm_path; rejected seams block program generation (no partial programs) |
| No absolute paths, one config | package share paths, `repo_root` / `--job`, env `AWM_OCC_PYTHON`; `awm_params.yaml` = ChainParams defaults (tested) |
| Events at rest | WeldStart after the approach segment, WeldEnd after the weld segment; each segment ends at zero velocity |
| SI units | metres / radians / seconds inside; `_mm`, `_deg` only in field names that say so |
| Safety | mock_components only; no IP addresses; limits from config, none raised |

## 5. Decisions taken autonomously (please review)

1. **ROS 2 Jazzy in Docker** instead of Humble on the host (CLAUDE.md, change proposal CP-1).
2. **Partial seams** (`range_m`, A22 start / end) added to the seam schema; the pipe test part is
   welded in 3 arcs (CP-5). Arc limits come from a reach map of the NOMINAL cell.
3. **Joint path stored in path.json** (D07) and reused by M6 after an FK check (CP-6).
4. **KDL for Cartesian paths, pick_ik global for IK search** (two SRDF groups, CP-7).
5. **Velocity limits per segment kind**: weld segments run at the WPS speed and are checked
   against the configured joint limits; other moves against limits × `velocity_scaling` (0.2).
   The pipe arc S01 needs wrist_2 at 0.91 rad/s (29 % of 3.14 rad/s).
6. **Release scopes**: `simulation` (report PASS only, marked not valid for production) and
   `production` (also PASS dry-run and trial-weld records). Records are never generated by the
   software.
7. **Fixture at x = 0.42 m** (NOMINAL, `cell_nominal.yaml`), torch TCP from the Tac demo torch
   (TODO(OI-1)).
8. Repo-local git identity "Truong Hoang <truongxhoang@apillis.com>" (none was configured).

## 6. Open issues and limits

| # | Item | Impact / next step |
|---|---|---|
| 1 | All cell data is NOMINAL: torch model (OI-1), fixture and Pismo frame (OI-6), calibration (A51–A53) | Re-run the chain after measured data; the TCP consistency check stops planning when model and calibration differ |
| 2 | WPS files are SAMPLE, not qualified | Welding engineer to provide D05; production config must set `allow_sample_wps: false` |
| 3 | No laser head in D14 (OI-2) | LASER seams are rejected at A4 with this reason |
| 4 | Weave (R3), lap / butt / groove orientation (R3), single-face edges | rejected with a reason |
| 5 | OMPL air moves are random: cycle time varies by ±5 % between runs | acceptable for R1; a deterministic air-move strategy could be added in R3 |
| 6 | A released program's upstream files (path.json …) may be re-planned later; the released program then reads as "stale" against them | runtime (M8) must check program sha vs release.json, not upstream inputs |
| 7 | Replay checks mock hardware only (perfect tracking); URSim / real dry run not done | A72 on the robot (R1/R2) |
| 8 | No Qt Workbench; `real` and `workbench` launches not written | out of scope; API is ready (`awm_seam.EdgeCatalogue.pick`, `awm_cad.import_step`, …) |
| 9 | DES-003 xlsx: `#REF!` statuses, stray rows, D16 missing; DES-001 F09 undefined, figures missing | see change proposal CP-4 |
| 10 | `awm_dev` was created before `--init` was added (zombie processes after stopped launches, harmless) | recreate the container when convenient |

## 7. Blockers

None. No sudo, credentials or hardware were needed; no design conflict forced a stop (the schema
additions CP-5 / CP-6 are optional / new fields of the first schema version, OI-7).

## 8. Next steps (suggested)

1. Review and decide CP-1 … CP-8 at G2; update DES-001 to Rev 0.3.
2. Fill OI-1 / OI-6 (torch, fixture, frame drawings) and measure the cell (A51–A53, R2).
3. Replace SAMPLE WPS by the welding engineer's WPS library.
4. Qt Workbench on top of the existing Python API (A13 viewer, A21 picking).
5. M8 / M9 / M10 (runtime, PLC bridge) against `awm_interfaces` v0 and DES-002.
6. URSim dry run (A72) with the released simulation programs.
