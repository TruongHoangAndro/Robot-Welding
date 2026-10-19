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
