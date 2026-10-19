# Function Register — sheet "A9"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A9 - Inspect & feed back |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Inspect the weld, record the result and feed corrections back to seams, parameters or path. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | Welded part D12 Weld log |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | Acceptance criteria (drawing / WPS) |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D13 Inspection record Corrections to A2-A4 |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Operator Process / robot engineer |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A91 |  | A92 |  | A93 |  | A94 |  |  |  |  |  |  |
|  | Inspect weld | ► | Record result / NCR | ► | Feed back corrections | ► | Seam tracking / vision |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A91  Inspect weld | Visual and dimensional inspection (leg size, porosity, undercut, distortion). |  | Welded part |  | Acceptance criteria (drawing / WPS) |  | Inspection result |  | Operator Process / robot engineer | TBD | Not started | Phase 1 |
|  | A92  Record result / NCR | Save inspection result, raise NCR if rejected. |  | Inspection result D12 Weld log |  | QA procedure |  | D13 Inspection record |  | Operator | TBD | Not started | Phase 1 |
|  | A93  Feed back corrections | Correct seams, parameters or path (A2-A4) and re-release. |  | D13 Inspection record |  | Change control |  | Corrections to A2-A4 |  | Process / robot engineer | TBD | Not started | Phase 1 |
|  | A94  Seam tracking / vision | On-line seam tracking or vision correction. |  | Welded part |  | - |  | Path correction |  | Weld planning software (IPC, Ubuntu + ROS2) | TBD | Not started | Phase 2 |
