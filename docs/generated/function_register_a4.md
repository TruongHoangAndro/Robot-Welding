# Function Register — sheet "A4"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A4 - Plan weld path |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Turn seams into tool poses with torch angles, weave, approach / retract, IK and collision check. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D04 Seam list D06 Seam process parameters |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | D14 Cell model (URDF) Robot reach / joint limits |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D07 Weld path (part frame) |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Weld planning software (IPC, Ubuntu + ROS2) |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A41 |  | A42 |  | A43 |  | A44 |  | A45 |  |  |  |  |
|  | Discretize seam | ► | Set torch orientation & weave | ► | Add approach / retract moves | ► | Solve IK & check reach | ► | Check collisions |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A41  Discretize seam | Split each seam into points with a tool frame at each point. |  | D04 Seam list |  | Point spacing / tolerance |  | Seam points |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A42  Set torch orientation & weave | Work angle, travel angle, stick-out / focus offset, weave pattern. |  | Seam points D06 Seam process parameters |  | D05 WPS library |  | Tool poses |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A43  Add approach / retract moves | Approach, retract and air moves between seams. |  | Tool poses |  | Clearance rules |  | Complete path |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A44  Solve IK & check reach | Inverse kinematics, joint limits, singularities, best robot configuration. |  | Complete path |  | UR5e kinematics / joint limits |  | Joint path |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A45  Check collisions | Robot, torch, fixture and part collision check; export path. |  | Joint path |  | D14 Cell model (URDF) |  | D07 Weld path (part frame) |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
