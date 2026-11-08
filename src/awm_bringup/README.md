# awm_bringup

```bash
ros2 launch awm_bringup sim.launch.py [headless:=true]   # mock hardware + move_group (+ RViz)
```
Mock hardware only. The `real` and `workbench` launches of DES-001 are not part of this delivery;
a real-robot launch is run by a person only.

| Path | Content |
|---|---|
| `config/awm_params.yaml` | D16: the single parameter file of the chain (repo root, OCC interpreter, groups, planning, timing, verification) |
| `repo_template/cell/` | SAMPLE WPS (D05), NOMINAL calibration (D08, `active.json`), io_map placeholder (D11) |
| `awm_bringup.testing.SimLaunch` | runs the headless sim in a private ROS domain (tests, `run_chain --start-sim`) |

## G1.16: the repository cell model in `sim_mock` (`repo:=`)

```bash
ros2 launch awm_bringup sim_mock.launch.py repo:=<repo root> trajectory_controllers:=robot_positioner
ros2 run awm_bringup replay_program --program <repo root>/jobs/<job>/programs/P###/program.json
```
`repo:=` reads `<repo>/cell/model/frames.yaml` (base and positioner poses, positioner axis), `scene_manifest.json`
(collision scene, must be from the same export) and `head/` (H-CAD when `head.yaml` exists; without it only an
explicit `head_model:=hbox` is accepted). `base_*`, `positioner_*`, `cell_manifest`, `head_dir` together with
`repo:=` are refused; a missing or malformed file stops the launch before any node starts. Without `repo:=` the
launch behaves as before. Tests: `test_repo_args.py`, `test_sim_mock_repo_launch.py` (golden CP-60 repository, see "Golden data" below).

## G1.7: 7-joint controller and D09 replay (DES-001 §6.3 Option A1)

```bash
ros2 launch awm_bringup sim_mock.launch.py trajectory_controllers:=robot_positioner   # default: split
ros2 run awm_bringup replay_program --program <job>/programs/P###/program.json [--segments G002-G004]
```
`robot_positioner` spawns one `robot_positioner_trajectory_controller` (6 UR5e joints + `positioner_joint`, mock
hardware) instead of the split pair. The replay first checks that every ros2_control hardware component is
`mock_components/GenericSystem` and that it talks to the 7-joint controller (else exit 2), then sends each D09 v2.1 `robot_positioner` segment unchanged (no time
scaling), refuses a segment whose start differs from `/joint_states` by more than `start_state_tolerance_deg`
(exit 2; the refused segment is never sent, earlier segments stay executed: status `partial`, or `refused`
when nothing moved), checks the end state, logs the tracking error and prints `replay {json}`. Weld events are
listed, never executed. Config: `config/program_replay.yaml`. Golden launch test: `AWM_CELL_STEP=<cell STEP> python3 -m pytest -s src/awm_bringup/test/test_sim_mock_7axis_launch.py`.

## G1.22: golden data from the cell STEP alone

```bash
scripts/dx "python3 scripts/golden_rebuild.py --cell-step <cell STEP> [--out <repo root>] [--force]"
```
Rebuilds the golden repository of part 564082 (S10 cell import with roles, positioner axis, head STEP + TCP teach,
cell save, job STEP via `make-job-step`, S1 import, job roles / axis, cross-check, CP-62 override, J1 / J2 seams,
plan, program, S4 files) through an in-process `awm_api` with the real CLIs. Every pick comes from
`test/data/golden_564082.yaml`; any mismatch stops the rebuild. Output (default `build/golden_564082`, or
`AWM_GOLDEN_OUT`) is reused while the cell STEP, the data file, the script and the engineering sources are unchanged;
`golden_summary.json` holds the key numbers. The golden tests of awm_seam, awm_path, awm_program and awm_bringup get
the repository from the session fixture `golden` (`test/conftest.py`), which runs the script once when `AWM_CELL_STEP`
is set and skips with `AWM_CELL_STEP not set` otherwise.
