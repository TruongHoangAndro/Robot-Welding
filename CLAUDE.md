# APLSMN601 Automated Welding Machine — ROS 2 workspace

Robotic MIG / laser welding cell. UR5e on the Pismo frame. The software imports a STEP model,
lets the user pick weld seams, plans the weld path offline, verifies it in simulation, then
replays the released program on the robot while the cell PLC switches the welder / laser.

Owner: Apillis Inc. · Project code: APLSMN601 · Phase 1 target: early Dec 2026

## Read first (source of truth)
@docs/reference/APLSMN601-DES-001_Software_Architecture_Rev0.2.md

Function definitions: `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx`
(exported to `docs/generated/function_register_*.md`). Reference demo: `docs/reference/ros2_nvidia_build-UR5E_WorkingDemo.zip`.
If code and the architecture document disagree, stop and ask. Do not silently change the design.
Note: DES-001 Rev 0.2 ADR-10 says Humble. That is superseded by the Docker/Jazzy setup below; a change
proposal goes in `docs/generated/DES-001_change_proposal.md`.

## Environment (Docker, fixed)
- Host: WSL Ubuntu 22.04. NO ROS on the host. Do not install ROS on the host and do not change the host OS.
- ROS 2 Jazzy runs ONLY in the dev container `awm_dev` (image `ros2-jazzy-hardware-accelerated:latest`).
- Workspace path is identical on host and container: `$HOME/ros2/aplsmn601_ws` (bind mount).
- Every ROS / colcon / pytest-with-ROS command runs as:
  `docker exec awm_dev bash -lc "source /opt/ros/jazzy/setup.bash && cd $HOME/ros2/aplsmn601_ws && <cmd>"`
  Helper: `scripts/dx <cmd>` (wraps the line above).
- Container `ros2_jazzy_dev_env` is the original demo: never stop, modify, exec into for writing, or delete it.
- Containers `supabase_*` belong to another project: never touch them.
- ROS isolation: `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `awm_dev`.
- Packages missing in the image: install inside `awm_dev` with `docker exec -u root awm_dev apt-get install -y ...`
  AND add the same line to `docker/Dockerfile` (reproducible image). Never install on the host with sudo.

## OCCT (ADR-02)
- pythonocc-core 8.0.1 lives only in host conda env `awm_occ` (Python 3.10), mounted read-only into `awm_dev`
  at the same path. Interpreter: `$HOME/miniforge3/envs/awm_occ/bin/python`, passed via env var
  `AWM_OCC_PYTHON` / ROS parameter `occ_python_cmd`. Never hard-code it in source.
- B-Rep code lives only in the pure-Python package `awm_geometry` (no rclpy), run as
  `$AWM_OCC_PYTHON -m awm_geometry.cli <command> --job <dir>`. ROS nodes call it via subprocess.
- Never import OCC in a ROS node. Never activate conda in the ROS shell. The demo used OCC 7.x: check API changes.

## Package layout (DES-001 §4.2) — prefix `awm_`
| Package | Module | Phase 1 scope |
|---|---|---|
| awm_interfaces | — | msg / srv / action (DES-001 §7.1) |
| awm_core | M0 | data classes, JSON schemas, frame math, job repository API |
| awm_geometry | M1/M2 helper | OCCT B-Rep work, CLI only (conda env) |
| awm_description | D14 | URDF/xacro/SRDF + MoveIt config: UR5e + torch + Pismo frame + fixture |
| awm_bringup | — | launch `sim` (mock hardware + RViz); config YAML (D16) |
| awm_cad | M1 | STEP import node / CLI, part record |
| awm_seam | M2 | manual seam selection + attributes, seams.json |
| awm_process | M3 | WPS library, MIG / laser parameters per seam |
| awm_path | M4 | discretise, torch orientation, approach/retract, IK, collision, constant TCP speed |
| awm_program | M6 | path → base frame, trajectory segments + weld events |
| awm_verify | M7 | simulation replay, reports, release lock |

## Design rules (do not break)
1. Offline / online split: plan and verify offline. The runtime only replays released programs.
2. Data-driven: no part-specific code, seam IDs or poses in source. All part data lives in the job package
   (DES-001 §5.2) with `schema_version`, `created_by`, `created_at`, `inputs` (SHA-256).
3. Weld paths are stored in the `part` frame. Transform to `base` only in awm_program, using the active calibration.
4. A seam path is accepted only if 100 % reachable and collision-free. No silent fallbacks.
5. No absolute paths in source. Use package share paths, the repository root parameter, env vars. One config source (D16).
6. The PLC owns welder, laser and interlocks. Weld events happen only while the robot is at rest at segment boundaries.
7. Units: SI internally (m, rad, s). mm / deg only in UI and JSON fields whose names say so (e.g. `leg_mm`).

## Coding conventions
- Python 3 (ament_python) for engineering modules; C++17 only where DES-001 says so.
- Type hints, docstrings, rclpy / rclcpp logger (no print in nodes). Declared parameters with defaults.
- pytest per package; pure logic testable without ROS. JSON schema validation tests. English comments.

## Safety rules (mandatory)
- NEVER connect to a real robot, PLC, welder or laser. Only `use_fake_hardware:=true` / mock_components.
- No real IP addresses in code; config YAML only, with placeholders.
- Software never bypasses or emulates safety functions. Do not raise speed / acceleration limits.

## Files you must not modify
`docs/reference/**`, `~/ros2/welding_demo_ws/**`, container `ros2_jazzy_dev_env`, anything outside this workspace except `/tmp/`.
