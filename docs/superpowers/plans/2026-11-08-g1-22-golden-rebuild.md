# G1.22 (close-out C1, redefined 2026-11-08) — golden data rebuilt from the cell STEP alone

Owner: cell-sim · Branch: `g1-22-golden-rebuild` from GotoCode 84bfba6 · Source: `2026-11-07-g1-closeout.md` C1 as
redefined by Truong on 2026-11-08; DES-001 Rev 0.5 §5.2 (job package, repository root parameter), §5.3 (frames,
`tcp_laser`), §5.4, ADR-16 / CP-60 (cell and job CAD import with roles), CP-61 (TCP teach), CP-62 (override of
cross-check 4), CP-59 (sync mode), CLAUDE.md design rules 2, 4, 5 and the test rule "the only allowed skip is
`AWM_CELL_STEP not set`".

## Goal
One script rebuilds every golden data set from the cell STEP; the hand-made repositories (`.worktrees/spike_repo`,
`.worktrees/demo/golden_repo_g1`, `golden_repo_cp60`) and the variables `AWM_SPIKE_REPO`, `AWM_GOLDEN_REPO`,
`AWM_CP60_REPO`, `AWM_S4_GOLDEN_DIR` are no longer needed. Decision of the same day: the top-level duplicate
`ASSY, Welding tool:1` (OI-23) gets cell role `exclude`.

## Design
- `scripts/golden_rebuild.py --cell-step <STEP> [--out <root>] [--data <yaml>] [--force]`, runs in `awm_dev`.
  It drives an in-process `awm_api` app (FastAPI `TestClient`) with the default CLI commands, so the gates (CP-60,
  CP-62), run promotion and override carry-over are the browser's. Two user-placed files are made with
  `awm_geometry make-job-step` from the same STEP: the head STEP (the robot component, moved to the robot's own
  frame read from the cell component tree) and the job STEP (rotating set + part).
- Data file `src/awm_bringup/test/data/golden_part.yaml` (test data owned by cell-sim, next to the launch tests that
  use the result; `docs/generated` is the lead's documentation scope and not test input): users, tolerances, cell
  roles (+ collision override, contact tolerance), cell axis pick, head map rules, TCP teach pick and expected TCP,
  job components, roles, axis pick, expected cross-check reds, CP-62 reason, J1 / J2 joint ids, seam defaults,
  travel speed.
- Refusals: picked edge not the expected edge / radius (±0.001 mm), TCP off by > 0.01 mm, cross-check reds other than
  the expected, a joint missing / open / wrong length / not between part solids, coaxial flags other than expected, a
  rejected or partial seam, override not carried into D07 / D09, a non-empty output folder not made by the script.
- Idempotent: `golden_summary.json` stores a fingerprint (cell STEP, data file, script, engineering package sources
  without tests); same fingerprint = nothing done. File lock per output folder (parallel colcon packages).
- Output default `build/golden_repo` (git- and colcon-ignored), or `AWM_GOLDEN_OUT`. `s4_golden/` holds
  `robot_description.json`, `path.json`, `placement.json` for the web FK test.

## Tests
- `src/awm_bringup/test/test_golden_rebuild.py` (always on): every refusal rule, data file, head map, robot-frame
  transform, output folder rules, fingerprint, no part data in the script.
- Session fixture `golden` in `test/conftest.py` of awm_seam, awm_path, awm_program, awm_bringup; the golden tests
  (`test_joint_cli.py`, `test_golden_g13.py`, `test_golden_g15.py`, `test_sim_mock_repo_launch.py`,
  `test_sim_mock_7axis_launch.py`) take their repository from it.
- vitest `fk.test.ts`: change described for the web agent (not edited here).
- Full suite with `AWM_CELL_STEP` and `AWM_OCC_PYTHON` set; evidence `docs/generated/goal1/g1-3_golden_run.md` and
  `g1-5_golden_program.md` regenerated from the summary.
