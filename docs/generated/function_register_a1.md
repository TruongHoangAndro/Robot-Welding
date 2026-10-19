# Function Register — sheet "A1"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A1 - Import CAD model |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Read the customer STEP file and show the 3D part model. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D01 STEP file |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | Customer drawing & revision |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D02 B-Rep model D03 Part record |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Weld planning software (IPC, Ubuntu + ROS2) |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A11 |  | A12 |  | A13 |  | A14 |  |  |  |  |  |  |
|  | Load STEP file | ► | Check & heal model | ► | Display 3D model | ► | Create part record |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A11  Load STEP file | Read STEP AP203 / AP214 / AP242 into the CAD kernel. |  | D01 STEP file |  | Supported STEP versions |  | D02 B-Rep model |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A12  Check & heal model | Check units, scale, closed solids, small gaps; heal if needed. |  | D02 B-Rep model |  | Customer drawing & revision |  | D02 B-Rep model (validated) |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A13  Display 3D model | Show the part in the UI (rotate, zoom, pick edges). |  | D02 B-Rep model |  | - |  | 3D view (UI) |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
|  | A14  Create part record | Store part no., revision, file name and hash so programs are linked to the right model. |  | D01 STEP file |  | Document numbering rule |  | D03 Part record |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 1 |
