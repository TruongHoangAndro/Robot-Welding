# APLSMN601 Automated Welding Machine — ROS 2 workspace

Robotic **laser** welding cell. UR5e on the Pismo frame, IPG LightWELD 1000 (laser only in Phase 1; MIG moved to
Phase 2, process abstraction kept). The software imports a STEP model, lets the user pick weld seams in the 3D
view, plans the weld path offline, verifies it in simulation, then replays the released program on the robot while
the cell PLC switches the laser.

Owner: Apillis Inc. · Project code: APLSMN601 · Phase 1 target: early Dec 2026
Last update of this file: 2026-10-27 (Drive master copy of 2026-10-25, aligned with DES-001 Rev 0.4, DES-003 Rev 0.4,
TST-002 Rev 0.2; plus wiki rules and the decisions of 2026-10-27 below)

## Current stage: simulation only
Real hardware is on hold until the Sim gate (decision 2026-10-22; schedule proposal PM-004 v2, not yet accepted):
| Milestone | Content | Proposed date |
|---|---|---|
| Sim-0 Setup | Docker Jazzy + MoveIt2 + ur_robot_driver, conda pythonocc, Git + CI, interface contracts, **TST-002 spike** | 2026-10-30 |
| Sim-1 MoveIt mock | STEP → Web 3D view → seam picking → path → IK / collision → playback (mock hardware), golden part 564082 | 2026-11-20 |
| Sim-2 URSim | Same trajectories through ur_robot_driver on URSim | 2026-12-04 |

## Read first (source of truth)
@docs/reference/APLSMN601-DES-001_Software_Architecture_Rev0.4.md
@docs/reference/APLSMN601-TST-002_Collision_Avoidance_Spike_Rev0.2.md
@docs/LLM_Wiki/wiki/overview.md

| Document | What it is |
|---|---|
| DES-001 Rev 0.4 | Software architecture: modules M0–M12, data items D01–D20, ADR-01…15, open items OI-1…21 |
| DES-003 Rev 0.4 | Function register (Excel): functions A11…A95 with input / output, phase and "Sim stage" column |
| TST-002 Rev 0.2 | Collision-avoidance feasibility spike: 5-day simulation test of the ψ × IK-branch planner on part 564082, criteria C1–C8, Go / Pivot / No-Go |
| DES-004 Rev 0.1 | As-built review of the Tac Huynh demo, findings F01–F15 (not in this workspace) |

Function definitions: `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.4.xlsx` (column "Sim stage" = scope per stage;
exported to `docs/generated/function_register_*.md` with `scripts/export_function_register.py`).
Reference demo: `docs/reference/ros2_nvidia_build-UR5E_WorkingDemo.zip`.
Cell CAD: `docs/3D/APLSMN601_Automated Weldment System.step` (Fusion export, mm, **Y up in the file**).
Golden part: component `564082-Assembly-3 parts` inside that file (pipe nipple + solid + elbow, joints J1 and J2).
`docs/3D/564082_Solid.step` is an older fused copy (solid + elbow only); do not use it as the golden part.
Older revisions in `docs/reference/` (DES-001 Rev 0.2, DES-003 Rev 0.1) are kept for history only.
If code and the architecture document disagree, stop and ask. Do not silently change the design.
Change proposals go in `docs/generated/DES-001_change_proposal.md`.

## Decisions of 2026-10-27 (Truong; to be written into DES-001 Rev 0.5 — see change proposal)
- **Z axis = positive upwards.** `world` Z points up. The seam frame Z is the bisector of the two face normals and points
  out of the material, i.e. up from the joint towards the head. The beam / tool axis points the opposite way (−Z of the
  seam frame, into the joint). TST-002 D1 "Z = −bisector" is read as the beam axis, not the seam frame.
- **TCP and nozzle contact (OI-13, wiki L-001): left to the software team.** Chosen in the change proposal; treat it as an
  Assumption until IPG automation documentation (OI-2) confirms it.
- **Rotary positioner (1 axis) is in Phase 1** (CP-20; supersedes ADR-11 "Phase 2 slot"). Phase 1 modes: C = robot at
  rest, positioner turns during emission (coaxial seams); B = rotate, stop, weld. No synchronised 7-axis motion.
  ROS 2 controls the positioner motor (`awm_positioner`, ros2_control, same controller manager as the UR driver).
- **Cell and fixture geometry come from Fusion STEP assemblies** with marker components `REF_*` (CP-23). The UR5e
  model itself always comes from `ur_description`.
- Sim-1 ends at the simulation report; Sim-2 replays a simulation-scope release (CP-17). Placement flow CP-22.
  Calibration before planning (CP-18). Tacks by hand (CP-19). Seam picking by Engineer only (CP-21).

## Environment (Docker, fixed)
- Host: WSL Ubuntu 22.04. NO ROS on the host. Do not install ROS on the host and do not change the host OS.
- ROS 2 Jazzy (ADR-10, Jazzy from Phase 1) runs ONLY in the dev container `awm_dev` (image `ros2-jazzy-hardware-accelerated:latest`).
- Workspace path is identical on host and container: `$HOME/ros2/aplsmn601_ws` (bind mount).
- Every ROS / colcon / pytest-with-ROS command runs as:
  `docker exec awm_dev bash -lc "source /opt/ros/jazzy/setup.bash && cd $HOME/ros2/aplsmn601_ws && <cmd>"`
  Helper: `scripts/dx <cmd>` (wraps the line above).
- Container `ros2_jazzy_dev_env` is the original demo: never stop, modify, exec into for writing, or delete it.
- Containers `supabase_*` belong to another project: never touch them.
- ROS isolation: `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `awm_dev`.
- Packages missing in the image: install inside `awm_dev` with `docker exec -u root awm_dev apt-get install -y ...`
  AND add the same line to `docker/Dockerfile` (reproducible image). Never install on the host with sudo.
- Python packages not in apt (e.g. `ur-analytic-ik`): install in a venv inside `awm_dev`, record in `docker/Dockerfile`.
  Never `pip install` into the system Python (PEP 668 on Ubuntu 24.04).

## OCCT (ADR-02, ADR-15: CAD runs as a separate process)
- pythonocc-core 8.0.1 lives only in host conda env `awm_occ` (Python 3.10), mounted read-only into `awm_dev`
  at the same path. Interpreter: `$HOME/miniforge3/envs/awm_occ/bin/python`, passed via env var
  `AWM_OCC_PYTHON` / ROS parameter `occ_python_cmd`. Never hard-code it in source.
  [TBC] DES-001 ADR-15 and TST-002 E1 name pythonocc-core 7.9 / Python 3.12. Do not change the env; list the
  difference in the change proposal until the team decides.
- B-Rep code lives only in the pure-Python package `awm_geometry` (no rclpy), run as
  `$AWM_OCC_PYTHON -m awm_geometry.cli <command> --job <dir>`. ROS nodes / `awm_api` call it via subprocess and exchange files
  (STEP in; JSON / glb / collision mesh out).
- Never import OCC in a ROS node. Never activate conda in the ROS shell. The demo used OCC 7.x: check API changes.

## Package layout (DES-001 Rev 0.4 §4.2) — prefix `awm_`
| Package | Module | Scope |
|---|---|---|
| awm_interfaces | — | msg / srv / action (DES-001 §7.1); length fields named `*_mm` |
| awm_core | M0 | data classes, JSON schemas, frame math, job repository API, **the only mm ↔ m adapter** (ADR-14) |
| awm_geometry | M1/M2 helper | OCCT B-Rep work, CLI only (conda env) |
| awm_description | D14 | URDF/xacro/SRDF + MoveIt config: UR5e + LightWELD head and mount (+ fiber / wire-feed conduit near the head) + Pismo frame + rotary positioner joint (Phase 1, CP-20) + fixture + clamps |
| awm_bringup | — | launch `sim` (mock hardware, Sim-1), URSim (Sim-2), `web`; config YAML (D16); RViz for developer debug only |
| awm_cad | M1 | STEP import, split solids, heal, tessellate (glb view model D20 + collision mesh), part record, placement / collision scene D19 |
| awm_seam | M2 | manual seam picking by edge ID + attributes, seams.json (no automatic seam detection — removed from scope, ADR-03) |
| awm_process | M3 | WPS library, LightWELD presets per seam (wobble is a head parameter, no robot weave) |
| awm_path | M4 | seam frame, head orientation, approach/retract, ψ × IK-branch graph search, collision, time parameterisation (ADR-12, see below) |
| awm_program | M6 | path → base frame, trajectory segments + weld events |
| awm_verify | M7 | simulation playback, reach / collision / cycle-time report, release lock |
| awm_api | UI backend | FastAPI + rclpy, REST + WebSocket (DES-001 §7.4), roles (ADR-09) |
| awm_web | UI front-end | React + TypeScript + react-three-fiber + urdf-loader (ADR-09) |
| Later (Sim-2 / hardware) | M8–M12 | awm_cell, awm_executor, awm_plc_bridge (`sim` mode first, ADR-13; LightWELD only), awm_positioner (ros2_control, Phase 1, CP-20), awm_logger, awm_inspect |

## Path planning and collision avoidance (ADR-12, TST-002)
Process path and collision are solved together, not in sequence. Algorithm (DES-001 §4.3):
1. Discretise the seam by arc length (1–3 mm). Seam frame: X = tangent, Z = bisector of the two face normals (n1, n2 from B-Rep, pointing out of the material = up), Y = Z × X.
2. Nominal head orientation from joint type and WPS; allowed tolerance band per seam.
3. Per point, sample tool yaw ψ (10° steps, 0–350°) × the 8 UR5e IK branches. Drop states that are in collision,
   beyond joint limits, near a singularity, or below the clearance padding.
4. Ladder-graph search (dynamic programming) over all points: one state per point, no IK-branch change, bounded joint
   step, cost = joint motion + deviation from nominal + low-clearance penalty + wrist-3 travel (fiber / conduit wrap).
5. No path → reject the seam and report the failing point and the colliding link / object. Never fall back to a partial path.
6. Approach / retract 30–50 mm along the beam axis; OMPL air moves between seams; all collision-checked.

Libraries:
- MoveIt2 `move_group` + FCL (planning scene, `check_state_validity`, distance queries), OMPL for air moves.
- IK enumeration: analytic UR IK returning all 8 solutions (`ur-analytic-ik` from PyPI, or a port of `ur_kin.cpp`).
  `pick_ik` returns one solution only — not enough for the sweep. `ur_kinematics` is ROS 1 only. `opw_kinematics` does not fit the UR wrist.
- UR packages from `ros-jazzy-ur` (ur_robot_driver, ur_description, ur_moveit_config). UR does not ship a collision-avoidance library.
- Fallback if TST-002 ends in Pivot / No-Go: Tesseract + tesseract_planning (Descartes, TrajOpt, OMPL) — do not start it without a decision.

Collision model (DES-001 §5.4): FCL works on meshes only, never on B-Rep. Part mesh chord ≤ 0.1 mm near seams, each
triangle keeps its `face_id`; ACM lets only the TCP link touch the seam faces; 3 mm padding on head, nozzle, mount,
conduit, fixture, clamps. Until IPG CAD arrives use the simplified head model of TST-002 §3.1 and mark it as an assumption.

## Design rules (do not break)
1. Offline / online split: plan and verify offline. The runtime only replays released programs.
2. Data-driven: no part-specific code, seam IDs or poses in source. All part data lives in the job package
   (DES-001 §5.2) with `schema_version`, `created_by`, `created_at`, `inputs` (SHA-256).
3. Weld paths are stored in the `part` frame. Transform to `base` only in awm_program, using the active calibration.
4. A seam path is accepted only if 100 % reachable and collision-free. No silent fallbacks.
5. No absolute paths in source. Use package share paths, the repository root parameter, env vars. One config source (D16).
6. The PLC owns the LightWELD, the laser interlocks and the wire feeder (ADR-04). Weld events happen only while the robot
   is at rest at segment boundaries. Weld segments start only at 100 % speed scaling.
7. Units (ADR-14): **mm, degrees, seconds in all project data** — JSON schemas, `awm_interfaces` messages, UI, reports.
   Third-party ROS packages (URDF, TF, MoveIt2, ur_robot_driver) stay in m / rad (REP 103). Convert only through the
   single adapter in `awm_core`, which has its own unit tests. No ad-hoc `* 1000` / `/ 1000` anywhere else.
8. Simulate first (ADR-13): every hardware interface has a simulated driver with the same interface; switching to
   hardware is a launch parameter, not a code change.

## Coding conventions
- Python 3 (ament_python) for engineering modules; C++17 only where DES-001 says so (M9).
- Type hints, docstrings, rclpy / rclcpp logger (no print in nodes). Declared parameters with defaults.
- pytest per package; pure logic testable without ROS. JSON schema validation tests. English comments.

## Safety rules (mandatory)
- NEVER connect to a real robot, PLC, LightWELD or laser. Only `use_fake_hardware:=true` / mock_components / URSim.
- Never write code that commands laser emission outside the PLC `sim` mode. The LightWELD is a Class 4 laser.
- No real IP addresses in code; config YAML only, with placeholders.
- Software never bypasses or emulates safety functions. Do not raise speed / acceleration limits.

## Files you must not modify
- `docs/reference/**`: do not edit content. Only add new revisions copied unchanged from the Drive master
  (allowed by Truong, 2026-10-27).
- `~/ros2/welding_demo_ws/**`, container `ros2_jazzy_dev_env`, anything outside this workspace except `/tmp/`.

## Project wiki (docs/LLM_Wiki)
- `docs/LLM_Wiki/` is the project knowledge wiki (schema: `docs/LLM_Wiki/PROJECT.md`). Use it for context; DES / TST documents in `docs/reference/` remain the design source of truth.
- Coding sessions (Claude Code opened at the workspace root): `docs/LLM_Wiki/**` is read-only. List the wiki updates needed at the end of the session instead of editing.
- Wiki sessions (Claude Code opened at `docs/LLM_Wiki/`) follow `docs/LLM_Wiki/PROJECT.md`.
- Never mix code and wiki changes in one commit; wiki commits start with `wiki:`.
