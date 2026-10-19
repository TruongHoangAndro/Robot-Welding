# APLSMN601 offline engineering chain — user guide

For process / robot engineers and developers running the chain A1 → A7 in simulation.
Everything runs in the dev container `awm_dev` (ROS 2 Jazzy). Mock hardware only: nothing here
connects to a robot, PLC, welder or laser.

## 1. One-time setup

```bash
cd ~/ros2/aplsmn601_ws
scripts/create_dev_container.sh          # creates awm_dev (idempotent), see docker/README.md
scripts/dx "colcon build --symlink-install"
```
`scripts/dx "<cmd>"` runs a command in `awm_dev` with ROS and the workspace sourced.
The OCC interpreter is the host conda env `awm_occ`, passed as `AWM_OCC_PYTHON`.

## 2. Run the whole chain in one command

```bash
scripts/dx "ros2 run awm_verify run_chain \
    --job  jobs_repo/jobs/1234_B \
    --step /path/to/1234_B.step --part-no 1234 --part-rev B \
    --picks /path/to/1234_B_seams.json \
    --approved-by 'Your Name' --start-sim"
```

| Step | Function | Output (in the job package) |
|---|---|---|
| A1 | STEP import, check & heal, mesh, part record | `model.step`, `part.json`, `cache/` |
| A2 | seams from the pick file | `seams.json` |
| A3 | process + WPS per seam | `process.json` |
| A4 | torch poses, reach and collision check | `path.json` (part frame) |
| A6 | program in the robot base frame | `programs/P###/program.json` |
| A7 | verification on mock hardware, release | `programs/P###/verify/report.json`, `release.json` |

- The repository (`jobs_repo/` above, i.e. `<job>/../..`) is created from the template on the first
  run: `cell/wps/` (SAMPLE WPS), `cell/calibration/active.json` (NOMINAL), `jobs/`, `logs/`.
- `--start-sim` starts `sim.launch.py headless:=true` in a private ROS domain for this run.
  Without it, start the simulation yourself (step 3).
- Without `--approved-by` the program is verified but not released.
- Exit code 0: complete; 2: stopped with a reason (seam rejected, no WPS, FAIL report); 1: error.

## 3. Step by step (with your own simulation window)

```bash
scripts/dx "ros2 launch awm_bringup sim.launch.py"            # RViz on WSLg; headless:=true for none
# in a second terminal:
P='--ros-args --params-file install/awm_bringup/share/awm_bringup/config/awm_params.yaml'
scripts/dx "ros2 run awm_core awm_repo init --repo jobs_repo --template install/awm_bringup/share/awm_bringup/repo_template"
scripts/dx "ros2 run awm_cad import_step --repo jobs_repo --step part.step --part-no 1234 --part-rev B"
scripts/dx "ros2 run awm_seam seam define --job jobs_repo/jobs/1234_B --seam-id S01 \
            --pick 0.0,0.005,0.010 --joint-type fillet --leg-mm 4 --sequence 1 \
            --material S235JR --thickness-mm 10 --process-hint MIG"
scripts/dx "ros2 run awm_process assign --job jobs_repo/jobs/1234_B --allow-sample-wps"
scripts/dx "ros2 run awm_path plan_path --job jobs_repo/jobs/1234_B $P"
scripts/dx "ros2 run awm_program generate --job jobs_repo/jobs/1234_B $P"
scripts/dx "ros2 run awm_verify verify --job jobs_repo/jobs/1234_B $P"
scripts/dx "ros2 run awm_verify release --job jobs_repo/jobs/1234_B --rev P001 --approved-by 'Name' --scope simulation"
```

## 4. Defining seams

Seams are picked on the B-Rep edges listed in `cache/edges.json` (ids `E<n>`, convexity, polyline).
A pick is a 3D point in the part frame (metres) near the edge, as a Workbench click would give.

Pick file (`--picks`, or `ros2 run awm_seam seam from-picks`):

```json
{"seams": [
  {"seam_id": "S01", "sequence": 1, "pick_m": [0.0, 0.005, 0.010],
   "joint_type": "fillet", "weld_side": "bisector", "leg_mm": 4.0,
   "process_hint": "MIG", "material": "S235JR", "thickness_mm": 10.0},
  {"seam_id": "S02", "sequence": 2, "pick_m": [0.0, 0.025, 0.012], "reversed": true,
   "start_pick_m": [-0.0125, -0.0217, 0.012], "end_pick_m": [-0.025, 0.0, 0.012],
   "joint_type": "fillet", "weld_side": "bisector", "leg_mm": 3.0,
   "process_hint": "MIG", "material": "S235JR", "thickness_mm": 2.5}
]}
```
- `weld_side`: `bisector` (torch on the bisector of the two faces) or a face id `F<n>` (the WPS work
  angle is measured from that face).
- `start_pick_m` / `end_pick_m` (optional, A22 start / end) weld only part of the edge chain, e.g. a
  pipe circle in several arcs. `reversed` flips the weld direction.
- A pick on a vertex shared by two edges is refused as ambiguous: pick in the middle of the edge.

## 5. Reading the results

- `path.json` → `rejected`: every seam that is not 100 % reachable and collision-free, with the
  reason (no partial seams are kept). Typical fixes: split the seam (start / end picks), weld in
  the other direction (`reversed`), change the part placement on the fixture.
- `verify/report.json`: checks `inputs`, `continuity`, `limits`, `events_at_rest`, `collision`,
  `replay`, cycle time and weld time per seam; `result` PASS / FAIL.
- `release.json` scope `simulation` is **not valid for production**. A production release needs:
  ```bash
  scripts/dx "ros2 run awm_verify record --template dry_run"          # form
  scripts/dx "ros2 run awm_verify record --job <job> --rev P001 --type dry_run --operator NAME \
              --result PASS --speed-override-pct 25 --seam S01=PASS"
  scripts/dx "ros2 run awm_verify record ... --type trial_weld ..."
  scripts/dx "ros2 run awm_verify release --job <job> --rev P001 --approved-by NAME --scope production"
  ```
- A released program directory is read-only. Any change (seam, WPS, calibration, path) makes the
  downstream files stale (hash check) and requires a new program revision.

## 6. Configuration (D16)

`awm_bringup/config/awm_params.yaml` is the single parameter file (repository root, OCC
interpreter, planning groups, sample spacing, approach distance, scalings, verification
tolerances). Cell geometry (Pismo frame, fixture, torch TCP) is in
`awm_description/config/cell_nominal.yaml`; the NOMINAL calibration must match it.

## 7. Tests

```bash
scripts/dx "colcon test && colcon test-result --verbose"
$HOME/miniforge3/envs/awm_occ/bin/python -m pytest src/awm_geometry/test   # OCC suite directly
```
The end-to-end test is `src/awm_verify/test/test_e2e_chain.py` (two generated parts + one
rejection case).
