# Function Register — sheet "A5"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A5 - Calibrate cell & locate part |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Calibrate TCP and frames, and locate the actual part on the fixture. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | Physical cell, fixture, part |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | Calibration procedure |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D08 Calibration data |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | UR5e robot + controller Process / robot engineer |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A51 |  | A52 |  | A53 |  |  |  |  |  |  |  |  |
|  | Calibrate TCP | ► | Calibrate fixture frame | ► | Locate part |  |  |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A51  Calibrate TCP | Torch / laser head TCP calibration on the robot. |  | Physical cell |  | Calibration procedure |  | TCP data |  | UR5e robot + controller Process / robot engineer | TBD | Not started | Phase 1 |
|  | A52  Calibrate fixture frame | Measure fixture / work frame relative to robot base (3-point method). |  | Physical cell, fixture |  | Calibration procedure |  | Fixture frame |  | UR5e robot + controller Process / robot engineer | TBD | Not started | Phase 1 |
|  | A53  Locate part | Check part position each cycle (touch sensing) or verify fixed jig position. |  | Part on fixture |  | Positioning tolerance |  | D08 Calibration data |  | UR5e robot + controller Process / robot engineer | TBD | Not started | Phase 1 |
