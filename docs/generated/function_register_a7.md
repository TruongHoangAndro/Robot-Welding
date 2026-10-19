# Function Register — sheet "A7"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A7 - Simulate & verify program |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Check the program in simulation, dry run and trial weld before release. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D09 Robot program |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | D14 Cell model (URDF) APLSMN601-REQ-002 Risk Assessment |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D15 Simulation report D10 Released program |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Weld planning software (IPC, Ubuntu + ROS2) UR5e robot + controller Process / robot engineer |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A71 |  | A72 |  | A73 |  | A74 |  |  |  |  |  |  |
|  | Simulate program | ► | Dry run on robot | ► | Trial weld on coupon | ► | Release program |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A71  Simulate program | URSim / RViz: reach, collisions, cycle time. |  | D09 Robot program |  | D14 Cell model (URDF) |  | D15 Simulation report |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A72  Dry run on robot | Run at reduced speed without arc / laser, check path on the real part. |  | D09 Robot program |  | APLSMN601-REQ-002 Risk Assessment |  | Dry-run result |  | UR5e robot + controller Process / robot engineer | TBD | Not started | Phase 1 |
|  | A73  Trial weld on coupon | Weld test coupon / first part and check quality. |  | D09 Robot program |  | D05 WPS library |  | Trial weld result |  | UR5e robot + controller MIG welder / IPG laser Process / robot engineer | TBD | Not started | Phase 1 |
|  | A74  Release program | Approve and lock the program revision for production. |  | D15 Simulation report Dry-run result Trial weld result |  | Approval by engineering |  | D10 Released program |  | Process / robot engineer | TBD | Not started | Phase 1 |
