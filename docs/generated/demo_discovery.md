# Demo discovery: hold_and_weld (Tac Huynh ROS 2 Jazzy demo) → APLSMN601 rewrite

Generated 2026-10-19. Read-only discovery. Nothing under `~/ros2/welding_demo_ws` was changed or built.

Sources read:
- `~/ros2/welding_demo_ws/haw_workspace/src/hold_and_weld/` (5 packages + gripper_sampler)
- `~/ros2/welding_demo_ws/haw_workspace/{Dockerfile,DEBUGGING_NOTES.md,toRunThis.txt,calibration_tools/}`
- `/tmp/ref_demo/ros2_nvidia_build-UR5E_WorkingDemo/` (demo_single_weld.py, demo_weld_cell.py, runbook.md, WeldDemo, Dockerfile)
- `docs/reference/APLSMN601-DES-001_Software_Architecture_Rev0.2.md` §9, §10

Path abbreviations used below:
- `HW/` = `~/ros2/welding_demo_ws/haw_workspace/src/hold_and_weld/`
- `APP/` = `HW/hold_and_weld_application/`
- `PLN/` = `HW/hold_and_weld_planning/hold_and_weld_planning/`
- `BR/` = `HW/hold_and_weld_bringup/`
- `DSC/` = `HW/hold_and_weld_description/`
- `WAS` = `APP/src/action_servers/welder_action_server.cpp`

---

## 1. Package map: demo → DES-001 §10 → new awm package

| Demo package | Nodes / executables | Launch files | Key deps (declared) | DES-001 §10 decision | New awm package |
|---|---|---|---|---|---|
| hold_and_weld_planning (ament_python) | `seam_generator` CLI (console_script, **not a ROS node**; writes JSON) | none | rclpy, geometry_msgs; setup.py: numpy, scipy, pyyaml. **Not declared:** pythonocc-core (`OCC`), trimesh, manifold3d, CGAL pybind module `mesh_intersection` (built by hand in Dockerfile) | OCCT loader + weld_planner + seam primitives: reuse, refactor. Mesh/CGAL extractor: keep for Phase 2 | `awm_cad` (occt/), `awm_path` (planning/weld_planner.py, core/), `awm_seam` plug-in (mesh/). Pure-geometry parts → `awm_geometry` (conda py3.10, pythonocc 8.0.1) |
| hold_and_weld_application (ament_cmake, C++) | `welder_server` (lifecycle `welder_action_server`, action `/trigger_welder`), `gripper_server`, `move_to_pose_server` (not launched anywhere), `dual_robot_coordinator` (lifecycle, srv `~/trigger_sequence`); scripts `add_collision_objects.py`, `magic_wand.py`, `visualize_grasps.py`, `finger_visualizer.py` | (launched from bringup) | rclcpp(_action,_lifecycle), moveit_ros_planning_interface, moveit_msgs, control_msgs, controller_manager_msgs, tf2(_geometry_msgs), eigen, Ceres (find_package only, not in package.xml), urdf, kdl_parser, yaml-cpp, nlohmann_json, gz-transport13, gz-msgs10 | welder_action_server: reuse core, refactor (split planning → `awm_path`, execution → `awm_executor`); coordinator: replace; gripper: park | `awm_path` (IK/Cartesian/approach logic), `awm_executor` (replay), `awm_cell` (sequencer). Action defs → `awm_core` / interfaces |
| hold_and_weld_description | `scripts/publish_welder_description.py` (not installed; absolute paths) | none | rclcpp, xacro, urdf. **Not declared:** `ur_description` (used via `$(find ur_description)`), gz_ros2_control | Reuse, simplify (UR5e + torch + Pismo frame + fixture only) | `awm_description` |
| hold_and_weld_bringup | none | 13 launch files (§2) | launch, launch_ros, ros_gz_sim, ros_gz_bridge, controller_manager, JSB, JTC, moveit_ros_move_group, moveit_simple_controller_manager, moveit_planners_ompl, rviz2, RSP, tf2_ros, xacro. **Not declared:** `pick_ik`, `ur_description` | Reuse, extend (add `real` launch with ur_robot_driver/URSim; fix deps; no absolute paths) | `awm_bringup` (+ MoveIt config parts → `awm_description`) |
| hold_and_weld_gripper_sampler (C++) | `grasp_finder_node` (not launched) | none | rclcpp, eigen, yaml-cpp, tinyxml2, embree, fcl, opencascade | Park (Phase 2 branch) (F10) | — |
| calibration_tools (44 .py scripts present; DES says 47) | standalone scripts (IK seed search, per-seam tilt/roll sweeps, collision checks) | — | numpy/trimesh/rclpy; absolute `/workspace/...` paths | Do not reuse | Replaced by generic reach search (`awm_path`/`awm_verify`) |
| /tmp/ref_demo orchestration | `WeldDemo` (bash), `demo_single_weld.py`, `demo_weld_cell.py`, `record_demo.py`, `run_weld_demo_with_record.py` | drive `system_bringup` / `app_welder_server` via `docker exec` | docker, `ros2 action send_goal /trigger_welder`, `ros2 service call /dual_robot_coordinator/trigger_sequence` | not listed (orchestration) | Replace with `awm_bringup` launch + `awm_verify` scripted tests |

Note: no `awm_geometry` / `awm_process` / `awm_program` / `awm_verify` equivalent exists in the demo (DES "Not in demo — New", F11–F15).

---

## 2. Launch file inventory (`BR/launch/`)

- `system_bringup.launch.py` — full dual-robot cell: includes sim_gazebo (dual_robot.xacro + controllers.yaml), sim_spawn_objects, controllers_spawn, moveit_move_group, moveit_rviz, sim_spawn_objects (planning scene only), app_welder_server, app_gripper_server, app_coordinator.
- `welder_bringup.launch.py` — welder-only: sim_gazebo (robot2_welder.xacro + robot2_controllers.yaml, spawn_robot=false), own `ros_gz_sim create` spawner with `-J` safety-pose joints from `config/tasks/welding.yaml`, objects, controllers, move_group, rviz, welder server.
- `sim_gazebo.launch.py` — `gz_sim.launch.py` with world, `ros_gz_bridge` (clock + `/world/default/model/<name>/joint_state`→`/joint_states`), robot_state_publisher (xacro with `controller_config_file:=`), optional spawner.
- `sim_spawn_objects.launch.py` — spawns workpiece/weldment in Gazebo (`ros_gz_sim create`), TF bridges, and/or `add_collision_objects.py` (planning scene + ACM edit).
- `controllers_spawn.launch.py` — spawner chain JSB → robot1_arm_controller → gripper_controller → robot2_arm_controller (OnProcessExit). `robot_type` arg declared but **unused**: welder-only runs still try robot1/gripper spawners.
- `moveit_move_group.launch.py` — move_group with kinematics/joint_limits/ompl/moveit_controllers yaml, `allowed_start_tolerance: 0.50` (line 116). `robot_description_file` arg declared (line 50) but **ignored**: always loads `dual_robot.srdf` (line 69).
- `moveit_rviz.launch.py` — RViz with MoveIt plugin via a bash supervisor `ExecuteProcess` (health/TF timeouts), default `weld_only.rviz`; loads `dual_robot.srdf` (line 175).
- `app_welder_server.launch.py` — LifecycleNode `welder_server` (name `welder_action_server`) + auto configure/activate; passes dual_robot.srdf + kinematics.yaml.
- `app_gripper_server.launch.py` — LifecycleNode `gripper_server` + auto transitions.
- `app_coordinator.launch.py` — LifecycleNode `dual_robot_coordinator` + auto transitions.
- `gripper_bringup.launch.py` — gripper-only cell (gazebo, objects, controllers, move_group, rviz, gripper server).
- `magic_wand.launch.py` — RSP (robot1_gripper.xacro), static TF, move_group, rviz, add_collision_objects, `magic_wand.py` (interactive teach tool).
- `finger_visualizer.launch.py` — same base as magic_wand + `finger_visualizer.py` (grasp visualisation).

---

## 3. hold_and_weld_planning

### 3.1 Modules

| Module | Purpose |
|---|---|
| `seam_generator.py` | CLI: load YAML (`load_urdf_config`) → `JobPlanner.plan_job()` → `export_to_json` |
| `planning/job_planner.py` | Pipeline orchestrator; modes `auto`/`mesh`/`occt`/`assembly`; `_plan_job_assembly` holds hard-coded pipe-seam logic (F03) |
| `planning/weld_planner.py` | `WeldPlanner`: seam points + two normal sets → torch poses (spacing, work/travel angle, gap) |
| `occt/occt_loader.py` | `OCCTLoader`: STEP/IGES → `TopoDS_Shape`, `package://` resolution, 4×4 world transform |
| `occt/occt_generator.py` | `OCCTGenerator`: URDF collision primitives (box/cylinder/sphere) → fused OCCT compound |
| `occt/seam_extractor_occt.py` | `SeamExtractorOCCT`: face-pair proximity + fuzzy Boolean common → seam edges, wall normals, line/arc/polyline classification |
| `occt/__init__.py` | OCC imports wrapped in `try/except ImportError` → classes become `None` (silent degradation, added per DEBUGGING_NOTES §2) |
| `core/line_segment.py`, `core/arc_segment.py` | Pure geometry primitives (length, tangent, point_at, midpoint) |
| `core/seam.py` | `Seam`: wraps Line/ArcSegment + `config` dict + generated `poses` |
| `mesh/mesh_loader.py` | STL load, `package://`, manifold3d conversion |
| `mesh/shell_generator.py` | URDF primitives → watertight manifold3d shells |
| `mesh/seam_extractor.py` | CGAL corefinement intersection (pybind `mesh_intersection`), chain segments, normals from mesh faces |
| `mesh/path_creator.py` | Corner detection, spline smoothing, line/arc merge/classification |
| `mesh/mesh_intersection.cpp` + `CMakeLists.txt` | pybind11 + CGAL corefinement module (built outside colcon) |
| `urdf/urdf_processor.py` | xacro processing, `package://` resolution, collision transforms |
| `utils/path_utils.py` | YAML config loader, JSON export, auto output path |

### 3.2 OCC API calls used (Python)

| Module | API |
|---|---|
| occt_loader.py | `STEPControl_Reader` (`ReadFile`, `TransferRoots`, `OneShape`), `IGESControl_Reader`, `IFSelect_RetDone`, `TopoDS_Shape.IsNull`, `gp_Trsf.SetValues` (3×4), `BRepBuilderAPI_Transform(shape, trsf).Shape()` |
| occt_generator.py | `BRep_Builder.MakeCompound`/`Add`, `TopoDS_Compound`, `BRepPrimAPI_MakeBox(gp_Pnt, dx, dy, dz)`, `BRepPrimAPI_MakeCylinder(gp_Ax2, r, h)`, `BRepPrimAPI_MakeSphere(r)`, `gp_Ax2`, `gp_Dir`, `gp_Pnt`, `gp_Trsf`, `BRepBuilderAPI_Transform`, `BRepAlgoAPI_Fuse` |
| seam_extractor_occt.py | `TopExp_Explorer` (FACE, EDGE), `topods.Face`, `topods.Edge`, `BRepExtrema_DistShapeShape` (`IsDone`, `Value`), `BRepAlgoAPI_Common(face, face)` + `SetFuzzyValue` + `Build`, `BRepAdaptor_Curve` (`FirstParameter`, `LastParameter`, `Value`, `GetType`, `Line`, `Circle`, `IsClosed`), `GeomAbs_Line/Circle/Plane`, `gp_Lin.Distance`, `gp_Circ.Location/Radius/Axis`, `BRepBuilderAPI_MakeVertex`, `topexp.MapShapesAndAncestors` + `TopTools_IndexedDataMapOfShapeListOfShape` (`Contains`, `FindFromKey`), `TopTools_ListIteratorOfListOfShape`, `TopoDS_Shape.IsSame`, `Orientation()`/`TopAbs_REVERSED`, `brepgprop.VolumeProperties` + `GProp_GProps.CentreOfMass`, `BRepAdaptor_Surface` (`GetType`, `Plane().Axis().Direction()`), `BRep_Tool.Surface`, `ShapeAnalysis_Surface.ValueOfUV`, `GeomLProp_SLProps` (`IsNormalDefined`, `Normal`), `gp_Dir.Reverse` |

Not used by the demo (so nothing to port, but needed by `awm_path` for the rewrite): `BRepMesh_IncrementalMesh`, `GCPnts_*` (arc-length sampling), `BRepBndLib`, `BRepTools`, `HashCode`.

### 3.3 pythonocc-core 7.8+/OCCT 8.0 compatibility

Probed in `~/miniforge3/envs/awm_occ` (pythonocc **8.0.1.2**) with a scratch script; results:

| API | Status in 8.0.1 | Action |
|---|---|---|
| `topods_Face`, `topods_Edge` (flat functions) | **Removed** (missing) | Use `topods.Face(...)`, `topods.Edge(...)`. Demo already uses `topods.*` |
| `brepbndlib_Add`, `breptools_Write`, `brepgprop_VolumeProperties`, `topexp_MapShapesAndAncestors`, `BRep_Tool_Surface` | Still present, emit `DeprecationWarning` ("deprecated since pythonocc-core 7.7.1") | Use the static-class forms `brepbndlib.Add`, `breptools.Write`, `brepgprop.VolumeProperties`, `topexp.MapShapesAndAncestors`, `BRep_Tool.Surface`. Demo already uses the class forms |
| `TopoDS_Shape.HashCode(upper)` | **Removed** (AttributeError; OCCT 7.9 removal) | Use `hash(shape)` / `shape.IsSame()` / `TopTools_IndexedMapOfShape` / `TopTools_IndexedDataMapOfShapeListOfShape`. Demo does not use HashCode |
| `TopTools_ListIteratorOfListOfShape` | Present and works | `TopTools_ListOfShape` is also directly iterable in Python (`list(m.FindFromKey(e))` works). Prefer iteration |
| `TopTools_IndexedDataMapOfShapeListOfShape.Contains/FindFromKey` | Works | — |
| `BRepAlgoAPI_Common(a,b)` + `SetFuzzyValue` + `Build` | Works (4 edges on a box-on-box test) | The 2-arg constructor already runs the Boolean; fuzzy value set afterwards is applied only by the second `Build()`. Rewrite: use default ctor, `SetArguments`/`SetTools`, `SetFuzzyValue`, then `Build()` once |
| `BRep_Tool.Surface` + `ShapeAnalysis_Surface.ValueOfUV` + `GeomLProp_SLProps` | Works | — |
| `STEPControl_Reader.ReadFile` on missing file | Returns status 2 (not RetDone), no exception | Keep status check |
| `Standard_Integer`/`Standard_Real` type wrappers | Not used by demo | — |
| `OCC.Core.Interface.Interface_Static.CVal('xstep.cascade.unit')` | Returns empty string in 8.0.1 | Do not rely on it; enforce units explicitly (see 3.4) |

### 3.4 Units (mm vs m)

- OCCT STEP/IGES readers return geometry in **millimetres** (OCCT default target unit). `OCCTLoader` does **not** rescale; it applies `world_transform` whose translation is in **metres** (`job_planner._pose_to_matrix`, YAML `world_pose.xyz`).
- `WeldPlanner` works in **metres** (`gap_m = gap_mm/1000`, `waypoint_spacing_m`, `weld_offset_m`), `SeamExtractorOCCT.tolerance = epsilon = 1e-3` (implicitly 1 mm if metres).
- So the OCCT STEP path mixes mm geometry with m offsets — latent unit bug (never exercised: config uses `mode: assembly` with an STL, DEBUGGING_NOTES §2 switched off OCCT).
- Assembly/mesh path guesses units: `scale_factor = 0.001 if max(extents) > 10.0 else 1.0` (`PLN/planning/job_planner.py:189`). URDF mesh uses `scale="0.001 0.001 0.001"` (`DSC/urdf/environment/testweldment.urdf.xacro:8`).
- Rewrite: `awm_geometry` should store part geometry in mm (CAD native), convert explicitly to m at the `awm_path` boundary, and record the unit in the job package.

### 3.5 weld_planner algorithm (`PLN/planning/weld_planner.py`)

Inputs per seam (from extractor): `smoothed_points` (N×3, m), `normals_main`, `normals_secondary` (N×3), `is_edge_joint`. Parameters (`HW/hold_and_weld_planning/config/urdf_welding_conf.yaml`): `work_angle_deg 45`, `travel_angle_deg 5`, `gap_mm 35`, `weld_offset_mm 3`, `num_smooth_points 40`, `waypoint_spacing_mm` default 10.

Discretisation:
- Extractor samples the OCCT edge at `num_smooth_points` **uniform in curve parameter** u ∈ [First, Last] (`seam_extractor_occt.py:150-158`). Uniform in arc length only for lines/circles; edge orientation (`TopAbs_REVERSED`) is ignored, so travel direction is arbitrary.
- `_sample_by_distance` (`weld_planner.py:137-162`): greedy selection, keeps index i when accumulated polyline distance ≥ spacing, then resets; always adds last point. Spacing is therefore ≥ 10 mm, not exact. Waypoints are existing samples, not re-interpolated.

Orientation per sampled index (`weld_planner.py:164-228`):
```python
t  = normalize(p[i+1] - p[i-1])                     # central difference (one-sided at ends)
perp = normalize(cross(n_main, t));  away = -perp if dot(perp, n_sec) > 0 else perp
if is_edge_joint: z0 = away;     lean = -n_main;  gap_dir = n_main
else:             z0 = -n_main;  lean = away;     gap_dir = normalize(-away + n_main)   # (gap_dir unused below)
# base frame
z = normalize(z0); y = normalize(cross(z, t)); x = cross(y, z)
# work angle: rotate about x (tangent) by ±work_angle toward lean
s = sign(dot(cross(z, lean), x)) or 1
R_w = Rot(s * work_angle * x);  z, y = R_w z, R_w y
# travel angle: rotate about y (binormal) by +travel_angle
R_t = Rot(travel_angle * y);    x, y, z = R_t x, R_t y, R_t z
if z[2] < 0: z, y = -z, -y          # force "from above" (note: leaves frame left-handed)
position   = p[i] + gap_m * z       # TCP target (35 mm stand-off along +z)
weld_point = p[i] + weld_offset_m * z
R = [x | y | z] (columns) -> quaternion (scipy, xyzw)
```
- **Convention**: pose **+z points away from the seam** (out of the workpiece, back up the torch). `position` = seam point + gap along +z. This matches the URDF TCP `robot2_wire_tip`, whose +z points back into the torch (nozzle direction = −z, see §5).
- Work angle is measured from the base direction (`-n_main` for fillet/flat, `away` for edge joints); travel angle is a fixed-sign rotation about binormal (push/pull depends on arbitrary travel direction).
- Bug: the `z[2] < 0` flip negates z and y but not x → det(R) = −1; scipy `from_matrix` then returns the nearest proper rotation silently (`weld_planner.py:214-216`).
- Bug: `gap_offset_direction` is computed then never used (`weld_planner.py:181-199`); offset is always along final z.
- Post-processing (`job_planner._generate_poses`): splits seams where the x-axis (tangent) reverses (dot < 0) into sub-seams with ≥3 poses.

Assembly-mode orientation actually used in the shipped demo (`PLN/planning/job_planner.py:296-411`) is a **different, hand-tuned formula** per hard-coded pipe seam:
```python
r_hat  = normalize(r - dot(r, t) t)  (or project on axis_dir if perp_mode=='axis'), r = p - seam_center
normal = normalize(cos(tilt) * r_hat + sin(tilt) * axis_dir)        # z axis
if 'roll' in cfg: x = cos(roll) t + sin(roll) normalize(cross(z, t)); y = cross(z, x); x = cross(y, z)
else:             y = normalize(cross(z, ref)); x = cross(y, z)
quat sign continuity: if dot(q, q_prev) < 0: q = -q
tcp = p + gap_m * z;  weld_point = p + weld_offset_m * normal
```
Per-seam `tilt`, `axis_dir`, `roll`, `gap_mm` (45/35), `start` index shift, `ik_seed` are literals (`job_planner.py:299-306`), resampled to 33 points by arc length (`np.interp`, `job_planner.py:317-322`).

Approach/retract offsets: not computed in the planner; done at runtime in `WAS` (`approach_offset_z`, §4).

---

## 4. welder_action_server.cpp

Config (`APP/include/hold_and_weld_application/action_servers/welder_action_server.hpp:56-68` defaults; overridden by `BR/config/tasks/welding.yaml` via `load_config_from_yaml` `WAS:459-522`):

| Key | Header default | welding.yaml (line) |
|---|---|---|
| welder_group_name | robot2_welder_arm | robot2_welder_arm (1) |
| json_file | "" (→ newest *.json) | **absolute** `/workspace/haw_workspace_working/.../urdf_welding_conf_pipe_joints.json` (2) |
| approach_offset_z | 0.1 m | 0.02 m (18) |
| retract_offset_z | — | 0.02 (19) — **never read** |
| cartesian_path_threshold | 0.95 | **0.35** (20) — F01 confirmed |
| cartesian_step_size (eef_step / max_step) | 0.01 m | 0.005 m (21) |
| velocity_scaling | 0.3 | 0.3 (22) |
| max_ompl_planning_attempts / validation / cartesian retries | 3 / 3 / 2 | 5 / 3 / 3 (26-28) |
| use_approach_validation | true | false (29) |
| safety_pose.joint_positions | — | (6-15) read only by launch, **not by the server** |

Flow:
1. `on_configure` (`WAS:116-304`): waits for `/compute_cartesian_path` and `/controller_manager/list_controllers`; loads YAML; creates internal node `welder_moveit_internal` + `MoveGroupInterface(robot2_welder_arm)`, `setPlanningTime(10)`, `setNumPlanningAttempts(10)`, `setEndEffectorLink("robot2_wire_tip")` (`WAS:216`); optionally builds Ceres IK + `ApproachValidator` on chain `robot2_base_link`→`robot2_wire_tip` (`WAS:243-244`); creates action `trigger_welder`; worker thread.
2. `execute_weld` (`WAS:754-1054`): JSON from `config_.json_file` or `find_latest_json()` (newest mtime in `<ws>/src/hold_and_weld/hold_and_weld_application/trajectories`, derived by walking up 4 parents from install share, `WAS:563-582`); `load_seams_from_json` reads `seams{id:{poses[{position, quaternion(xyzw)}], ik_seed, approach_offset_z}}` (`WAS:622-712`, `json_to_pose` hpp:323-361 normalises q, no frame/unit field); filter by goal `seam_ids`; optional `apply_rotation` about world Z (`WAS:1668-1709`; comment says orientation is *not* rotated, code *does* rotate it `WAS:1694-1707`).
3. Per seam: approach (`move_to_seam_boundary`, up to `max_cartesian_retries`, seeds from JSON `ik_seed`, then random reseed `reseed_joint_seed` `WAS:1711-1744`) → Cartesian weld (`execute_cartesian_path`) with re-approach retries (j6 ±2π wrap shift, reseed, `WAS:956-979`) → retract (`move_to_seam_boundary(poses.back())`, fallback `retract_collision_free`) → `move_to_safety_pose`. Failures are collected; result `success = failed_seams.empty()`.

Approach (`move_to_seam_boundary`, `WAS:1255-1554`):
- Target = `ref + approach_offset * (q * +Z)` — code computes `torch_direction = q*(0,0,-1)` and subtracts it (`WAS:1269-1275`), i.e. backs off along pose **+z** (away from the work).
- Deterministic branch: `setFromIK(weld pose, "robot2_wire_tip", 5 s)` seeded from current state or `ik_seed`, then `setFromIK(approach pose)` warm-started from it (`WAS:1332-1335`); branch check |Δq1|,|Δq2| > 0.5 rad → re-seed (`WAS:1343`); joint unwrapping to seed (`WAS:1353-1367`); optional j6 ±2π (`WAS:1376-1393`); `setJointValueTarget(q_approach)`.
- Fallback: pose goal if IK fails (`WAS:1424-1427`) and after 3 failed OMPL attempts (`WAS:1454-1458`). Goal tolerances 1 mm / 0.01 rad. OMPL plan → optional approach validator → execute.

Cartesian weld (`execute_cartesian_path`, `WAS:1556-1666`):
- Preferred: `/compute_cartesian_path` request with `max_step = cartesian_step_size (0.005)`, **`jump_threshold = 0.0` (disabled)** (`WAS:1580`), **`avoid_collisions = false`** (`WAS:1591`), velocity/accel scaling 0.3, start_state = approach joint solution (`WAS:1597-1614`), 30 s timeout.
- Fallback (F07): `move_group_->computeCartesianPath(waypoints, step, trajectory)` from current state (collision-checked default) (`WAS:1641-1645`).
- Acceptance: `fraction < cartesian_path_threshold` → fail (`WAS:1647`); with welding.yaml that is **35 %** → a 35 % partial weld path is executed as success (F01). No time parameterisation for constant TCP speed; MoveIt's default time parameterisation with velocity scaling only (F12).

Retract / safety (F07 fallbacks):
- `retract_collision_free` (`WAS:1056-1195`): Cartesian move along +z with `avoid_collisions=false`, `jump_threshold 0`; accepts **fraction ≥ 0.50** (`WAS:1148`); then rewrites trajectory point 0 to current joints and unwraps steps (`WAS:1170-1184`) without re-validation.
- `move_to_safety_pose` (`WAS:1197-1253`): hard-coded joints `{-2.558, -1.442, -0.956, -2.793, 0.957, 0.735}` (`WAS:1199`); if OMPL fails, executes a **blind 20-step smoothstep joint interpolation over 4 s with no collision check** (`WAS:1217-1249`).
- `allowed_start_tolerance: 0.50` rad in move_group (`BR/launch/moveit_move_group.launch.py:116`) masks start-state mismatch.

Weld orientation and tool frame:
- Tool frame = `robot2_wire_tip` (SRDF chain tip, EEF link). JSON quaternions are interpreted directly as wire_tip orientation in the planning frame (`world`). Pose +z away from work; nozzle points along −z.
- No TCP calibration input; torch geometry is from the xacro only.

Hard-coded in WAS (F03/F08): EEF/tip link names (`WAS:216, 244, 1332, 1335, 1347`), base link (`WAS:243, 1434`), safety joints (`WAS:1199`), retract acceptance 0.50 (`WAS:1148`), 30 s service deadlines (`WAS:1121, 1618`), branch-jump 0.5 rad (`WAS:1343`), IK timeouts 5 s/1 s, planning time 10 s / attempts 10 (`WAS:213-214`), safety planning 5 s/5 (`WAS:1202-1203`), RNG seed `0x5EED` (`WAS:1720`), trajectory directory layout (`WAS:571-572`). Seam ids are not in WAS; they are in the coordinator and planner (§7).

---

## 5. hold_and_weld_description: robot2 (UR5e + torch)

Chain: `DSC/urdf/robot2_welder.xacro` → `robots/ur5e_arm_prefix.xacro` (`xacro:ur_robot` from **ur_description** `ur_macro.xacro`, `tf_prefix=robot2_`, `safety_limits=false`, ur5e default_kinematics/joint_limits/physical/visual yaml) → `robots/ur5e_arm_ros2_control.xacro` + `end_effectors/welding_torch_prefix.xacro`.

Placement: `ur5e_arm_macro parent="world" prefix="robot2_" x=1.80 y=-0.5 z=0.50 yaw=3.14159` (`robot2_welder.xacro:21`; same in `dual_robot.xacro:36`). Initial joint xacro args default `[0, -1.5708, 1.5708, -1.5708, -1.5708, 0]`.

Torch mount: `robot2_tool0_to_torch` fixed, parent `robot2_tool0`, child `robot2_torch_mount`, `xyz="0 0 0" rpy="3.14 0 0"` (`robot2_welder.xacro:32-36`; 3.14 not π → 1.6 mrad error).

`welding_torch_prefix.xacro` (all primitives, **no meshes**):

| Link | Visual/collision geometry | Origin in link |
|---|---|---|
| `${prefix}torch_mount` | cylinder r 0.04, l 0.03 | 0 |
| `${prefix}torch_handle` | cylinder r 0.025, l 0.16 | z −0.08 |
| `${prefix}torch_neck` | cylinder r 0.015, l 0.10 | z −0.05 |
| `${prefix}torch_nozzle` | cylinder r 0.01, l 0.06 | z −0.03 |
| `${prefix}weld_point` | none (frame) | — |
| `${prefix}wire_tip` | none (frame) — **TCP** | — |

| Joint (fixed) | Parent → child | xyz | rpy |
|---|---|---|---|
| mount_to_handle | torch_mount → torch_handle | 0 0 −0.015 | 0 0.5236 0 (30°) |
| handle_to_neck | torch_handle → torch_neck | 0 0 −0.16 | 0 0.2618 0 (15°) |
| neck_to_nozzle | torch_neck → torch_nozzle | 0 0 −0.10 | 0 0 0 |
| nozzle_to_weld_point | torch_nozzle → weld_point | 0 0 −0.06 | 0 0 0 |
| weld_point_to_wire_tip | weld_point → wire_tip | 0 0 −0.015 (15 mm stick-out) | 0 0 0 |

Computed TCP (with π instead of 3.14): `tool0 → wire_tip`: **xyz = (−0.2037, 0, 0.2773) m, rpy = (π, −π/4, 0)**; `tool0 → weld_point`: xyz = (−0.1931, 0, 0.2667). Total torch bend 45° (30° + 15°). Wire direction (out of nozzle) = wire_tip −z = (−0.707, 0, 0.707) in tool0. Inertials: 0.3/0.4/0.2/0.1 kg. Gazebo materials only.

ros2_control (`robots/ur5e_arm_ros2_control.xacro`): `<ros2_control name="robot2_ur5e_system" type="system">`, hardware plugin **`gz_ros2_control/GazeboSimSystem`** (line 9). No `mock_components/GenericSystem`, no `ur_robot_driver` variant. Per joint: command `position` (min/max ±6.35, elbow ±3.20), state `position` (with `initial_value`) + `velocity`. Gazebo plugin `gz_ros2_control-system` with `<parameters>${controller_config_file}</parameters>` (`robot2_welder.xacro:38-46`). The `ur_robot` macro is called without ros2_control-related args; the hardware interface comes from this custom block (verify against the installed ur_description version that `ur_macro.xacro` does not emit a second `<ros2_control>` tag).

Controllers (`DSC/config/robot2_controllers.yaml`): `update_rate 100`, `robot2_arm_controller` = `joint_trajectory_controller/JointTrajectoryController` (position command; position+velocity state), `joint_state_broadcaster`.

SRDF `DSC/config/robot2_welder.srdf`:
- group `robot2_welder_arm`: `<chain base_link="robot2_base_link" tip_link="robot2_wire_tip"/>`
- virtual joint `robot2_virtual_joint` fixed world→robot2_base_link
- group_state `home` = `[0, −1.5708, 1.5708, −1.5708, −1.5708, 0]`
- end_effector `welding_torch` parent `robot2_tool0` (no EEF group defined)
- disabled collisions: 55 hand-written pairs (Adjacent/Never) for UR links and torch links vs arm; **not used at runtime** — every launch loads `dual_robot.srdf` (329 disable entries, same `robot2_welder_arm` chain, plus robot1 groups whose links do not exist in the welder-only URDF).
- Additional ACM at runtime from `add_collision_objects.py:290-316`: allows `base_link` (table object) vs **all** robot2 links, `child_link` (weldment) vs `robot2_wire_tip` **and `robot2_upper_arm_link`** (broader than torch-only → F07).

Meshes:
- UR5e: `package://ur_description/meshes/ur5e/{visual/*.dae, collision/*.stl}` (seen in generated `DSC/urdf/robot2_welder.urdf:69-...`). Not in the workspace; require `ros-jazzy-ur-description` (installed only in the ref-demo image `/tmp/ref_demo/.../Dockerfile:90`, not declared in any package.xml → F05).
- Workpiece: `DSC/meshes/weldment/WeldmentAssy-Welment_001.stl` and `TestWeldment.stl` **exist**. Also gp25/ar2010 meshes (robot1, Phase 2).
- `DSC/urdf/robot2_welder.urdf` is a stale xacro output generated from `/workspace/haw_workspace_working/...` (line 3).

---

## 6. MoveIt config (`BR/config/moveit/`)

- `kinematics.yaml`:
  ```yaml
  robot1_arm:        {kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin, search_resolution: 0.005, timeout: 5.0, attempts: 10}
  robot2_welder_arm: {kinematics_solver: pick_ik/PickIkPlugin, kinematics_solver_timeout: 0.05, kinematics_solver_attempts: 3, mode: local}
  ```
  pick_ik params used: only `mode: local`. All other pick_ik params at defaults (e.g. `position_scale`, `rotation_scale`, `position_threshold`, `orientation_threshold`, `cost_threshold`, `minimal_displacement_weight`, `gd_max_iters`, `approximate_solution_*`). Note `setFromIK(..., 5.0)` in WAS overrides the 0.05 s timeout per call. `pick_ik` is not declared in any package.xml (F05); ref-demo image installs `ros-jazzy-pick-ik` (`Dockerfile:92`).
- `joint_limits.yaml` (UR5e): all six `robot2_*` joints `max_velocity 3.14159`, `max_acceleration 5.0` (has_* true). No jerk limits. Position limits come from ur_description `joint_limits.yaml` (safety_limits=false).
- `moveit_controllers.yaml`: `MoveItSimpleControllerManager`; controllers `robot1_arm_controller`, `gripper_controller`, `robot2_arm_controller` (FollowJointTrajectory, `action_ns: follow_joint_trajectory`, default true; robot2 joints `robot2_{shoulder_pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3}_joint`).
- `ompl_planning.yaml`: planning_plugins `ompl_interface/OMPLPlanner`; request adapters ResolveConstraintFrames, ValidateWorkspaceBounds, CheckStartStateBounds, CheckStartStateCollision; response adapters AddTimeOptimalParameterization, ValidateSolution, DisplayMotionPath; `start_state_max_bounds_error 0.05`; `robot2_welder_arm`: RRTConnect (default)/RRT/EST, `longest_valid_segment_fraction 0.005`, `enforce_constrained_state_space: true`, **`projection_evaluator: joints(robot2_joint_1,robot2_joint_2,robot2_joint_3)` — joint names do not exist on the UR5e** (leftover from GP25/AR2010; line 47).
- move_group launch extras: `moveit_manage_controllers: true`, `allowed_execution_duration_scaling 1.2`, `allowed_goal_duration_margin 0.5`, `allowed_start_tolerance 0.50`, planning scene monitor publishes everything; `robot_description` from topic.

---

## 7. Absolute paths, hard-coded seam ids / poses / IPs

Absolute paths (F06):
- `BR/config/tasks/welding.yaml:2` — `json_file: /workspace/haw_workspace_working/src/hold_and_weld/hold_and_weld_application/trajectories/urdf_welding_conf_pipe_joints.json`
- `PLN/planning/job_planner.py:433-434` — writes output JSON to `/workspace/haw_workspace_working/{src,install}/...urdf_welding_conf_pipe_joints.json` (side effect of planning)
- `DSC/scripts/publish_welder_description.py:45, 114-115, 150, 152` — URDF, trajectories, weldment STL under `/workspace/haw_workspace_working/...`
- `DSC/urdf/robot2_welder.urdf:3` — generated-from path
- `WAS:563-582` — trajectory dir derived from install layout (`<ws>/src/hold_and_weld/...`), assumes colcon layout
- `HW/../../Dockerfile` (build step) `cd ${ROS_WS}/src/hold_and_weld/src/hold_and_weld/.../mesh` and `toRunThis.txt` (`/ws/src/...`)
- calibration_tools: 30+ scripts hard-code `/workspace/haw_workspace_working/.../urdf_welding_conf_pipe_joints.json` (20×) and `.../WeldmentAssy-Welment_001.stl` (17×)
- ref demo: `demo_single_weld.py:19,25,66,73,77,85,94,123,290` (`/workspace/logs`, `/workspace/haw_workspace_working/install/...`); `demo_weld_cell.py:27,54`; `run_weld_demo_with_record.py:41-113` and `record_demo.py:16-182` (`/home/th/ros2_nvidia_build/...`, `/home/th/.gemini/...`); `entrypoint.sh:10-11`

IPs / ports:
- `/tmp/ref_demo/.../patch_openclaw_agenticros.py:31` — `ws://192.168.12.106:9090` (rosbridge)
- `/tmp/ref_demo/.../relay_mesh_visibility.py:50-53` — `http://127.0.0.1:8099/...`
- runbook: rosbridge `ws://172.17.0.1:9090`, ports 3978/18789/11434/3000 (agent stack, out of scope)
- No robot/PLC IP anywhere (no real-robot launch, F13).

Hard-coded seam ids (F02/F03):
- `APP/src/coordinator/dual_robot_coordinator.cpp:651` — pass 1 `{"seam_0","seam_1","seam_2"}`; `:656` — pass 2 `{"seam_3"}` with rotation. (Runbook text says seams 1-3 then seam_0 — code and docs disagree.) These ids match `trajectories/urdf_welding_conf_20260917_044326.json` (seam_0..seam_7) but **not** the JSON that welding.yaml actually selects (`seam_pipe_*`), so the coordinator path would abort "No seams matched".
- `PLN/planning/job_planner.py:229,244,256,269,281,294` — six pipe seam ids `seam_pipe_cross_small`, `_cross_drop`, `_cross_leg1`, `_leg1_elbow`, `_cross_leg2`, `_leg2_elbow`, extracted from STL bodies by index (`bodies[0..4]`), with literal geometry (`R_pipe = 50.80`, line 234).
- `PLN/planning/job_planner.py:299-306` — per-seam tilt/axis/roll/gap/start/**ik_seed joint vectors**.
- `/tmp/ref_demo/.../demo_single_weld.py:322-327` — same six ids; `:299` help text mentions seam_2/3/8/9.

Hard-coded poses / joints:
- `PLN/planning/job_planner.py:191-192` — assembly origin `xyz [0.137856, -0.117964, -0.153330]`, `rpy [-1.570796, -1.570796, 0]` (duplicated in `testweldment.urdf.xacro:6,15`).
- `HW/hold_and_weld_planning/config/urdf_welding_conf.yaml:5-13` — weldment world pose xyz (1.51, −0.5, 0.75), rpy (0, 0, 1.570796).
- `BR/config/objects/objects.yaml` and `APP/config/collision_objects/objects.yaml` (duplicate files) — table `base_link` at (1.51, −0.50, 0.25), weldment `child_link` at (1.51, −0.50, 0.75) q(0,0,0.7071,0.7071).
- `DSC/urdf/robot2_welder.xacro:21` — robot base (1.80, −0.5, 0.50, yaw π).
- `BR/config/tasks/welding.yaml:10-15` and `WAS:1199` — safety joints (duplicated).
- `APP/src/coordinator/dual_robot_coordinator.cpp:39-48` — robot2 safety positions (third, different set), cube reset (1.2, 0.3, 0.125), rotate 180° about (1.2, −0.5).
- `APP/action/TriggerWelder.action` — defaults `rotate_center_x 1.2`, `rotate_center_y -0.5`.

---

## 8. Findings F01–F15 → code locations

DES-001 references DES-004 for the F-list but only describes some findings; DES-004 itself was not available. Mapping below uses the DES-001 §9/§10 wording.

| Finding | DES-001 wording | Code location(s) |
|---|---|---|
| F01 | Runtime planning accepts 35 % Cartesian paths | `BR/config/tasks/welding.yaml:20` `cartesian_path_threshold: 0.35`; check `WAS:1647`; retract accepts 50 % `WAS:1148`; `jump_threshold = 0.0` `WAS:1108, 1580` |
| F02 | Seam IDs from code, not program | `dual_robot_coordinator.cpp:651, 656`; `TriggerWelder.action` rotate defaults |
| F03 | Hard-coded seams / assembly mode | `job_planner.py:163-444` (`_plan_job_assembly`: body indices, `R_pipe`, seam configs 299-306, origin 191-192); `urdf_welding_conf.yaml:3` `mode: assembly` |
| F04 | pythonocc not installed in Docker | `HW/../../Dockerfile` (no pythonocc; only libocct-*-dev for C++); `DEBUGGING_NOTES.md:18-28` (OCC import made optional, mode switched to mesh); `PLN/occt/__init__.py` try/except |
| F05 | Missing deps ur_description, pick_ik | `DSC/package.xml`, `BR/package.xml` (neither declared); used at `ur5e_arm_prefix.xacro:10-21`, `kinematics.yaml:10`; demo Dockerfile omits them, ref image adds them (`/tmp/ref_demo/.../Dockerfile:90,92`). Also undeclared: trimesh, manifold3d, Ceres (`APP/CMakeLists.txt:25`) |
| F06 | Absolute paths | §7 list (welding.yaml:2, job_planner.py:433-434, publish_welder_description.py:45/114/150, calibration_tools, ref scripts) |
| F07 | Unchecked fallbacks; ACM too broad | Cartesian `avoid_collisions=false` `WAS:1591, 1110`; fallback `computeCartesianPath` `WAS:1641-1645`; blind safety interpolation `WAS:1217-1249`; trajectory point rewrite `WAS:1170-1184`; pose-goal fallback `WAS:1424-1427, 1454-1458`; `allowed_start_tolerance 0.50` `moveit_move_group.launch.py:116`; ACM `add_collision_objects.py:290-316` (table vs whole arm, weldment vs upper arm); normals fallback `[0,0,1]` `seam_extractor_occt.py:533-544` |
| F08 | Config scattered / not from one source | Defaults in hpp:56-68 vs welding.yaml; safety pose in 3 places (welding.yaml:10-15, `WAS:1199`, coordinator.cpp:39-41); `retract_offset_z` unused; objects.yaml duplicated in bringup and application; `robot_description_file` and `robot_type` launch args ignored; all launches hard-load `dual_robot.srdf` |
| F09 | not described in DES-001 | — |
| F10 | Fixed Pismo frame / gripper, rail, GP25 out of Phase 1 | `dual_robot.xacro`, `robot1_gripper.xacro`, `gp25_*`, `linear_rail_prefix.xacro`, gripper server, gripper_sampler, coordinator rotation pass |
| F11–F15 | Not in demo (process params, calibration, events, verification, logging) | No WPS/process data; no TCP calibration (torch from xacro, §5); no weld on/off events; no verification beyond fraction; no weld log |
| F12 | Constant TCP speed | Not implemented: Cartesian trajectories use MoveIt default timing with `max_velocity_scaling_factor 0.3` (`WAS:1594-1595`); OMPL response adapter `AddTimeOptimalParameterization`; goal field `velocity_mps` exists only in unused `ExecuteWeld.action` |
| F13 | No real-robot launch | Only `gz_ros2_control/GazeboSimSystem` (`ur5e_arm_ros2_control.xacro:9`); no mock_components, no ur_robot_driver/URSim launch |
| F14 | No calibration hash | JSON has no frame/unit/calibration metadata (`json_to_pose` hpp:323-361); poses are in `world` frame, not part frame |

Additional defects worth carrying into the rewrite test list:
- `weld_planner.py:214-216` z-flip makes R left-handed.
- `seam_extractor_occt.py:150-158` ignores edge orientation; parameter-uniform not arc-length sampling.
- OCCT path mixes mm geometry with m offsets (§3.4).
- `ompl_planning.yaml:47` projection evaluator references non-existent `robot2_joint_1..3`.
- `apply_rotation` comment/code mismatch (`WAS:1689-1707`).
- `controllers_spawn.launch.py` spawns robot1/gripper controllers even for welder-only.
