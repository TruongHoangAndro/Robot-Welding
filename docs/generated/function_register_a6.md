# Function Register — sheet "A6"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A6 - Robot program |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Transform the path to the robot base frame and generate the program with weld / I/O events. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D07 Weld path (part frame) D08 Calibration data D06 Seam process parameters |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | D11 I/O & handshake signal list |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D09 Robot program |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Weld planning software (IPC, Ubuntu + ROS2) |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A61 |  | A62 |  | A63 |  | A64 |  |  |  |  |  |  |
|  | Transform path to robot frame | ► | Generate motion program | ► | Insert weld & I/O events | ► | Store & transfer program |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A61  Transform path to robot frame | Apply part -> fixture -> base transforms and TCP. |  | D07 Weld path (part frame) D08 Calibration data |  | - |  | Path in robot base frame |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A62  Generate motion program | URScript / UR Toolpath URCap or ROS2 trajectory (architecture to be decided). |  | Path in robot base frame |  | UR / ROS2 driver interface |  | Motion program |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A63  Insert weld & I/O events | Gas pre-flow, arc on / off, laser enable, wait for arc OK, crater fill. |  | Motion program D06 Seam process parameters |  | D11 I/O & handshake signal list |  | D09 Robot program |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A64  Store & transfer program | Save in program library with part record and send to the robot / IPC. |  | D09 Robot program D03 Part record |  | Program revision rule |  | D09 Robot program (stored) |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
