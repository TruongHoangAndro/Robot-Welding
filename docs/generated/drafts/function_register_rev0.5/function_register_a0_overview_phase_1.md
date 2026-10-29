# Function Register — sheet "A0 Overview Phase 1"

Exported from `docs/generated/drafts/APLSMN601-DES-003_Function_Register_Rev0.5_DRAFT.xlsx` (values only). Do not edit; re-export with `scripts/export_function_register.py`.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 |
|---|---|---|---|---|---|---|---|---|
| Team work packages P1-P4 (Phase 1), mapped to the functions. Rev 0.5: renamed from A1-A4 (clash with function IDs), input STEP (not STL), pose convention stated (DES-001 5.3). |  |  |  |  |  |  |  |  |
| Work package | Description | Input | Control | Output | Mechanism | Owner | Status ▼ | Phase |
| P1 (A1, A2) | STEP import -> seam list (part frame) | D01 STEP file (golden part: 564082 assembly from the cell CAD D21) | Customer drawing; axis convention DES-001 5.3 | D04 seams.json - per point: position X, Y, Z (mm) + seam frame quaternion (x, y, z, w); normals n1, n2 | awm_geometry / awm_cad / awm_seam | Nguyen Huu Thang Truong Hoang |  | Phase 1 |
| P2 (A4) | Seam list -> weld path (psi x IK-branch ladder, positioner Modes C / B) | D04, D06, D19, D08 | D14 cell model, D16 limits | D07 path.json - tool poses (mm + quaternion, part frame); joint path q1...q6 (deg) + positioner angle; IK branch (shoulder, elbow, wrist = +/-1) and psi per point | awm_path (MoveIt2, analytic UR IK) | Thanh Luu Khang Doan |  | Phase 1 |
| P3 (A6, A71) | Program + simulation | D07, D08 | D14, D19 | D09 program.json, D15 report; playback in the Web view / RViz | awm_program / awm_verify / awm_web | Thanh Luu Truong Hoang Nguyen Huu Thang |  | Phase 1 |
| P4 (A8) | ROS 2 to robot and positioner (Sim-2 on URSim, then hardware) | D10 released program (scope simulation in Sim-2) | D11, D16, UR safety configuration | Joint trajectories via ur_robot_driver + positioner controller; weld events to the PLC | awm_executor / awm_cell / awm_positioner / awm_plc_bridge | Thanh Luu Khang Doan |  | Phase 1 |
