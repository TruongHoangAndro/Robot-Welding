# G1.16 (close-out A2) — `sim_mock repo:=<root>` loads the repository cell model

Owner: cell-sim · Branch: `g1-16-sim-mock-repo` · Source: `2026-11-07-g1-closeout.md` A2 (audit B2 part 1),
DES-001 Rev 0.5 §5.2 (repository root is a parameter), §5.3 (frames), §5.4 (collision scene), ADR-13 (mock hardware),
ADR-16 / CP-60 (cell import writes `cell/model/`), CLAUDE.md design rule 5 (repository root parameter, one config source).

## Goal
`ros2 launch awm_bringup sim_mock.launch.py repo:=<repo root> [trajectory_controllers:=robot_positioner]` shows the
imported cell with nothing typed by hand: base and positioner poses from `cell/model/frames.yaml`, the collision scene
from `cell/model/scene_manifest.json`, the head from `cell/model/head/` (H-CAD when `head.yaml` is present).

## Choice: `repo:=` (not `cell_model_dir:=`)
- The repository root is already the one parameter of the chain (`awm_params.yaml` repo root, `awm_api`, `awm_program`
  read `<repo>/cell/model/...`); the S4 "Replay in RViz" (B1) passes the same root.
- Later the active calibration (`<repo>/cell/calibration/active.json`, D08) is read from the same root, not from
  `cell/model/`.

## Rules
1. `repo:=` set together with any of `base_xyz_mm`, `base_quat_xyzw`, `positioner_xyz_mm`, `positioner_quat_xyzw`,
   `positioner_axis_xyz`, `cell_manifest`, `head_dir` → refused (launch fails before any node starts).
2. Missing / malformed `frames.yaml` or `scene_manifest.json` → refused with the file and key named. The manifest must
   come from the same export as `frames.yaml` (same `inputs`, `frames_file: frames.yaml`).
3. Head: `head/head.yaml` present → `head_model:=hcad`, `head_dir:=<repo>/cell/model/head` (checked with
   `check_head_dir`). Absent → refused unless `head_model:=hbox` is passed explicitly. An explicit `head_model:=hbox`
   with a head present is allowed (logged).
4. Without `repo:=` everything stays as before (pose defaults identity, `cell_scene_nominal.yaml`, `hbox`).
5. Pure resolver `awm_bringup.cell_config.repo_launch_args` (no ROS), used by the launch.

## Tests (TDD)
- `test/test_repo_args.py` (always on, synthetic repo generated in `tmp_path`): values from frames.yaml (positioner
  axis in the axis frame), each refusal, head rules; one `ros2 launch` refusal subprocess test.
- `test/test_sim_mock_repo_launch.py` (gated by `AWM_CP60_REPO`, skipped with the reason otherwise): TF
  `world→base_link`, `world→positioner_axis`, `world→positioner_flange` = frames.yaml (≤ 1e-6 mm / deg); every manifest
  object in the planning scene; head = H-CAD (`tcp_laser` in `tool0` = head.yaml); 7-joint controller active and a
  synthetic replay through `replay_program` succeeds.

Commands: `scripts/dx "colcon build --symlink-install --packages-select awm_description awm_bringup && colcon test
--packages-select awm_description awm_bringup && colcon test-result --verbose"`; the gated test with
`AWM_CP60_REPO=<repo>`.
