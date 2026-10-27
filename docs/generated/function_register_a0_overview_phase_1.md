# Function Register — sheet "A0 Overview Phase 1"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.4.xlsx` (values only). Do not edit; re-export with `scripts/export_function_register.py`.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 |
|---|---|---|---|---|---|---|---|---|
| ID / Sub-function | Description | Input | Control | Output | Mechanism | Owner | Status ▼ | Phase |
| A1 | Import CAD to Seam Position | -File STL |  | Seam Position.json / - X,Y,Z,Q1,Q2,Q3 |  | Nguyen Huu Thang Truong Hoang |  |  |
| A2 | Seam Position to Path | Seam Position.json |  | Seam Path.json / - X,Y,Z,Q1,Q2,Q3,cf1,cf2,cf3 |  | Thanh Luu Khang Doan |  |  |
| A3 | Simulate | Seam Path.json / - X,Y,Z,Q1,Q2,Q3,cf1,cf2,cf3 |  | Visualize |  | Thanh Luu Truong Hoang Nguyen Huu Thang |  |  |
| A4 | ROS2 to Robot | Seam Path.json / - X,Y,Z,Q1,Q2,Q3,cf1,cf2,cf3 |  | Move(X,Y,Z,Q1,Q2,Q3,cf1,cf2,cf3) |  | Thanh Luu Khang Doan |  |  |
