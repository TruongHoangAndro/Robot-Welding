# Function Register — sheet "A0 Overview"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 |
|---|---|---|---|---|---|---|---|---|---|---|
|  | APLSMN601 Automated Welding Machine - A0 Functional Overview |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |
|  | Click a block or a function in the table to drill down.  Edit content only in 'Function Register'. |  |  |  |  |  |  |  |  |  |
|  | Main flow  (A1 → A4, then A6 → A9) |  |  |  | Swap to A2. |  |  |  | Sub-function status |  |
|  | A1 |  | A2 |  | A3 |  | A4 |  | Not started | 36 |
|  | Import CAD model | ► | Define weld seams | ► | Set weld process parameters | ► | Plan weld path |  | In progress | 0 |
|  |  |  |  |  |  |  |  |  | Done | 0 |
|  |  |  |  |  |  |  |  |  | On hold | 0 |
|  |  |  |  |  |  |  |  |  | Total | 37 |
|  |  |  |  |  |  |  | ▼ |  | Progress (Done) | 0 |
|  | A9 |  | A8 |  | A7 |  | A6 |  |  |  |
|  | Inspect & feed back | ◄ | Execute welding | ◄ | Simulate & verify program | ◄ | Robot program |  |  |  |
|  | ▲ Feedback: corrections to A2-A4 |  |  |  | D08 Calibration data  →  A6, A8 |  | ▲ |  |  |  |
|  |  |  |  |  | Supporting function |  | A5 |  |  |  |
|  |  |  |  |  |  |  | Calibrate cell & locate part |  |  |  |
|  | Function summary |  |  |  |  |  |  |  |  |  |
|  | ID (click to open) | Function |  | Input |  | Output |  | Mechanism | Status | Progress |
|  | ► A1 | Import CAD model |  | D01 STEP file |  | D02 B-Rep model D03 Part record |  | Weld planning software (IPC, Ubuntu + ROS2) |  | 0 |
|  | ► A2 | Define weld seams |  | D02 B-Rep model |  | D04 Seam list |  | Weld planning software (IPC, Ubuntu + ROS2) Process / robot engineer |  | 0 |
|  | ► A3 | Set weld process parameters |  | D04 Seam list |  | D06 Seam process parameters |  | Process / robot engineer |  | 0 |
|  | ► A4 | Plan weld path |  | D04 Seam list D06 Seam process parameters |  | D07 Weld path (part frame) |  | Weld planning software (IPC, Ubuntu + ROS2) |  | 0 |
|  | ► A5 | Calibrate cell & locate part |  | Physical cell, fixture, part |  | D08 Calibration data |  | UR5e robot + controller Process / robot engineer |  | 0 |
|  | ► A6 | Robot program |  | D07 Weld path (part frame) D08 Calibration data D06 Seam process parameters |  | D09 Robot program |  | Weld planning software (IPC, Ubuntu + ROS2) |  | 0 |
|  | ► A7 | Simulate & verify program |  | D09 Robot program |  | D15 Simulation report D10 Released program |  | Weld planning software (IPC, Ubuntu + ROS2) UR5e robot + controller Process / robot engineer |  | 0 |
|  | ► A8 | Execute welding |  | D10 Released program D08 Calibration data Workpiece |  | Welded part D12 Weld log |  | UR5e robot + controller PLC / safety relay MIG welder / IPG laser Operator |  | 0 |
|  | ► A9 | Inspect & feed back |  | Welded part D12 Weld log |  | D13 Inspection record Corrections to A2-A4 |  | Operator Process / robot engineer |  | 0 |
