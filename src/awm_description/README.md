# awm_description (D14 cell model)

UR5e (ur_description) + MIG torch (TCP `tcp_mig`, z = wire axis into the work) + Pismo frame +
fixture, ros2_control **mock hardware only**, SRDF and MoveIt configuration.

| File | Content |
|---|---|
| `config/cell_nominal.yaml` | single source of the NOMINAL geometry: frame, fixture pose, torch TCP (TODO(OI-1), TODO(OI-6)) |
| `urdf/awm_cell.urdf.xacro` | cell model; `torch_mig.xacro` builds the torch collision shape from the TCP (nozzle ends `stickout_m` before the TCP) |
| `srdf/awm_cell.srdf.xacro` | groups `weld_arm` (base_link -> tcp_mig) and `weld_arm_search` (same chain), home state |
| `config/kinematics.yaml` | `weld_arm`: KDL (Cartesian paths); `weld_arm_search`: pick_ik global (IK branch search) |
| `config/joint_limits.yaml`, `ompl_planning.yaml`, `moveit_controllers.yaml`, `ros2_controllers.yaml`, `initial_positions.yaml` | MoveIt / controllers |

The NOMINAL calibration in `awm_bringup/repo_template` must match `cell_nominal.yaml`
(checked by awm_bringup tests; the planner also compares the model TCP with the calibration TCP).

## Cell model v2 (laser): `urdf/awm_cell_v2.urdf.xacro`

UR5e + LightWELD head + `positioner_joint`, mock hardware. Head model by xacro / `sim_mock` launch arg `head_model`:

| `head_model` | Macro | Geometry | `tcp_laser` |
|---|---|---|---|
| `hbox` (default) | `urdf/lightweld_hbox.xacro` | parametric boxes / cylinders (TST-002 §3.1, Assumption) | tool0 +Z, mount_h + head_h + nozzle_len |
| `hcad` | `urdf/lightweld_hcad.xacro` | Fusion head CAD from `head_dir` (A18 `head.yaml` + STL): visual = full mesh, collision = convex hull | `beam_exit + nozzle_len_mm * beam_axis`, rotation from `tool0_T_tcp_mm` |

Both models have the same links (`head_mount`, `head_body`, `nozzle_body`, `nozzle_tip`, `conduit`, `tcp_laser`) and
joints, so the SRDF does not change. `head_dir` is an absolute folder (e.g. `<repo_root>/cell/model/head`), never a
path in source. Tests run the hcad cases on a synthetic head (`test/head_fixture.py`) and, with
`AWM_HEAD_DIR=<folder>`, also on the real export.

Conduit (G1.8): hbox places the capsule with the `conduit_*` args; hcad starts it at the head cable exit from
`head.yaml` (`cable_exit_tool0_mm` + `cable_axis_tool0`, or else the rear `estimates.tcp.geometry_estimate` that
export-head refused as the fiber / cable connector end) and runs it outward along the cable axis. Diameter and length
stay the `conduit_dia_mm` / `conduit_len_mm` args (Assumption, TST-002 §3.1). Without a cable exit in `head.yaml`
(automatic beam_axis export) the parametric `conduit_*` args are used (Assumption). The URDF records the source as
`<awm_conduit source="explicit|rear_estimate|parametric"/>`. With the cable keys the label follows `cable_exit_source` (`map` or null →
`explicit`, `rear_estimate` → `rear_estimate`); `awm_bringup.cell_config.conduit_source` uses the same rule.
