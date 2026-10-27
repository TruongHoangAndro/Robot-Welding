# Function Register — sheet "Data Items"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.4.xlsx` (values only). Do not edit; re-export with `scripts/export_function_register.py`.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 |
|---|---|---|---|---|---|---|---|
|  | Data Items (inputs / outputs between functions) |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  \|  Rev 0.4  \|  2026-10-22  \|  Prepared by: Truong Hoang  \|  Status: Draft |  |  |  |  |  |  |
| ID | Data item | Description | Format | Source (if external) | Produced by (# sub-functions) | Used by (# sub-functions) | Check |
| D01 | STEP file | Customer 3D model of the part | .stp / .step (AP203/214/242) | External (customer) |  |  |  |
| D02 | B-Rep model | Part geometry loaded in the CAD kernel | In memory / .brep |  |  |  |  |
| D03 | Part record | Part no., revision, file name, hash | JSON / database |  |  |  |  |
| D04 | Seam list | Seam geometry + attributes (joint, side, start / end, sequence), in mm | JSON |  |  |  |  |
| D05 | WPS library | Welding procedure specifications linked to LightWELD presets | Sheet / database | External (welding engineer) |  |  |  |
| D06 | Seam process parameters | Process and parameters per seam | JSON |  |  |  |  |
| D07 | Weld path (part frame) | Tool poses / joint path in part frame | JSON |  |  |  |  |
| D08 | Calibration data | TCP, fixture frame, part offset | JSON / UR installation |  |  |  |  |
| D09 | Robot program | Motion program with weld / I/O events | URScript / ROS2 trajectory |  |  |  |  |
| D10 | Released program | Approved, revision-locked program | URScript / ROS2 trajectory |  |  |  |  |
| D11 | I/O & handshake signal list | PLC <-> robot <-> LightWELD signals | Sheet (ICD, APLSMN601-DES-002) | External (design) |  |  |  |
| D12 | Weld log | Part, program rev, parameters, time, alarms | CSV / database |  |  |  |  |
| D13 | Inspection record | Inspection result and NCR | Sheet / form |  |  |  |  |
| D14 | Cell model (URDF) | Robot, torch, fixture model for IK / collision / simulation (positioner joint added in Phase 2) | URDF / xacro | External (design) |  |  |  |
| D15 | Simulation report | Reach, collision and cycle time result | PDF / screenshot / log |  |  |  |  |
| D16 | Cell & runtime configuration | Cell parameters: soft limits, speeds, timeouts, network, I/O mapping reference | YAML (ROS2 parameters) | External (design) |  |  |  |
| D17 | Run recording | Raw joints, I/O and status per run | rosbag2 |  |  |  |  |
| D18 | Part weld report | Report per part: seams, set vs actual parameters, faults, operator | PDF / CSV |  |  |  |  |
| D19 | Part placement & collision scene | Fixture -> part nominal transform, part orientation, collision scene objects | JSON + meshes |  |  |  |  |
| D20 | 3D view model | Per-face meshes and per-edge polylines with stable IDs + topology, for picking seams in the Web 3D view | glb + JSON |  |  |  |  |
|  | Data item IDs (D01 ...) must be written in the Input / Control / Output text of the Function Register so the counts work. |  |  |  |  |  |  |
