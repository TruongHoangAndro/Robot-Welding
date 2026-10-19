# Function Register — sheet "A8"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A8 - Execute welding |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Run the released program on the cell: PLC handshake, robot motion, MIG / laser welding, alarms, logging. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D10 Released program D08 Calibration data Workpiece |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | D11 I/O & handshake signal list UR safety configuration |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | Welded part D12 Weld log |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | UR5e robot + controller PLC / safety relay MIG welder / IPG laser Operator |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A81 |  | A82 |  | A83 |  | A84 |  | A85 |  |  |  |  |
|  | Load part & start cycle | ► | PLC - robot - welder handshake | ► | Run robot & weld | ► | Monitor & handle alarms | ► | Log weld data |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A81  Load part & start cycle | Load and clamp the part, operator start from outside the guard. |  | Workpiece |  | Operation manual |  | Cycle start request |  | Operator PLC / safety relay | TBD | Not started | Phase 1 |
|  | A82  PLC - robot - welder handshake | Ready, start, program select, arc on, arc OK, cycle end, alarm signals. |  | Cycle start request |  | D11 I/O & handshake signal list |  | Handshake signals |  | PLC / safety relay UR5e robot + controller MIG welder / IPG laser | TBD | Not started | Phase 1 |
|  | A83  Run robot & weld | Execute the released program with MIG or laser welding. |  | D10 Released program D08 Calibration data |  | UR safety configuration |  | Welded part |  | UR5e robot + controller MIG welder / IPG laser | TBD | Not started | Phase 1 |
|  | A84  Monitor & handle alarms | Arc fault, wire stick, collision, laser interlock, E-stop, communication loss. |  | Handshake signals |  | APLSMN601-REQ-002 Risk Assessment |  | Alarm / stop |  | PLC / safety relay UR5e robot + controller | TBD | Not started | Phase 1 |
|  | A85  Log weld data | Record part, program rev, parameters, time and alarms. |  | Handshake signals D10 Released program |  | - |  | D12 Weld log |  | Weld planning software (IPC, Ubuntu + ROS2) PLC / safety relay | TBD | Not started | Phase 1 |
