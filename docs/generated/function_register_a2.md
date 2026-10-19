# Function Register — sheet "A2"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A2 - Define weld seams |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Select and describe each weld seam on the model (manual in Phase 1, automatic later). |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D02 B-Rep model |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | Customer drawing (weld symbols) |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D04 Seam list |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Weld planning software (IPC, Ubuntu + ROS2) Process / robot engineer |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A21 |  | A22 |  | A23 |  | A24 |  |  |  |  |  |  |
|  | Select seam edges | ► | Define seam attributes | ► | Detect seams automatically | ► | Export seam list |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A21  Select seam edges | User picks edges / face pairs that form each seam (Phase 1 manual). |  | D02 B-Rep model |  | Customer drawing (weld symbols) |  | Seam geometry |  | Weld planning software (IPC, Ubuntu + ROS2) Process / robot engineer | TBD | Not started | Phase 1 |
|  | A22  Define seam attributes | Joint type (fillet, butt, lap, groove), weld side, start / end, sequence, leg size. |  | Seam geometry |  | Customer drawing (weld symbols) |  | Seam attributes |  | Process / robot engineer | TBD | Not started | Phase 1 |
|  | A23  Detect seams automatically | Find candidate seams from face adjacency / intersections. |  | D02 B-Rep model |  | Detection rules |  | Seam geometry (proposed) |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 2 |
|  | A24  Export seam list | Save seams with geometry and attributes. |  | Seam geometry Seam attributes |  | Seam list JSON schema |  | D04 Seam list |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
