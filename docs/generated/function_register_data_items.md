# Function Register — sheet "Data Items"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 |
|---|---|---|---|---|---|---|---|
|  | Data Items (inputs / outputs between functions) |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |
| ID | Data item | Description | Format | Source (if external) | Produced by (# sub-functions) | Used by (# sub-functions) | Check |
| D01 | STEP file | Customer 3D model of the part | .stp / .step (AP203/214/242) | External (customer) | 0 | 2 | OK |
| D02 | B-Rep model | Part geometry loaded in the CAD kernel | In memory / .brep |  | 2 | 4 | OK |
| D03 | Part record | Part no., revision, file name, hash | JSON / database |  | 1 | 1 | OK |
| D04 | Seam list | Seam geometry + attributes (joint, side, start / end, sequence) | JSON |  | 1 | 3 | OK |
| D05 | WPS library | Welding procedure specifications | Sheet / database | External (welding engineer) | 0 | 4 | OK |
| D06 | Seam process parameters | Process and parameters per seam | JSON |  | 1 | 2 | OK |
| D07 | Weld path (part frame) | Tool poses / joint path in part frame | JSON |  | 1 | 1 | OK |
| D08 | Calibration data | TCP, fixture frame, part offset | JSON / UR installation |  | 1 | 2 | OK |
| D09 | Robot program | Motion program with weld / I/O events | URScript / ROS2 trajectory |  | 2 | 4 | OK |
| D10 | Released program | Approved, revision-locked program | URScript / ROS2 trajectory |  | 1 | 2 | OK |
| D11 | I/O & handshake signal list | PLC <-> robot <-> welder / laser signals | Sheet (ICD, APLSMN601-DES-002) | External (design) | 0 | 2 | OK |
| D12 | Weld log | Part, program rev, parameters, time, alarms | CSV / database |  | 1 | 1 | OK |
| D13 | Inspection record | Inspection result and NCR | Sheet / form |  | 1 | 1 | OK |
| D14 | Cell model (URDF) | Robot, torch, fixture model for IK / collision / simulation | URDF / xacro | External (design) | 0 | 2 | OK |
| D15 | Simulation report | Reach, collision and cycle time result | PDF / screenshot / log |  | 1 | 1 | OK |
|  | Data item IDs (D01 ...) must be written in the Input / Control / Output text of the Function Register so the counts work. |  |  |  |  |  |  |
