# Function Register — sheet "A3"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 | c9 | c10 | c11 | c12 | c13 | c14 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  | A3 - Set weld process parameters |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |  |  |  |  |  |  |  |
|  | ◄ Back to A0 Overview |  | Edit in Function Register ► |  |  |  |  |  |  |  |  |  |  |
|  | Description | Choose MIG or laser per seam and assign welding parameters from the WPS. |  |  |  |  |  |  |  |  |  |  |  |
|  | Input | D04 Seam list |  |  |  |  |  |  |  |  |  |  |  |
|  | Control | D05 WPS library |  |  |  |  |  |  |  |  |  |  |  |
|  | Output | D06 Seam process parameters |  |  |  |  |  |  |  |  |  |  |  |
|  | Mechanism | Process / robot engineer |  |  |  |  |  |  |  |  |  |  |  |
|  | Status |  |  |  |  |  |  |  |  |  |  |  |  |
|  | Sub-functions (click a block to jump to its row below) |  |  |  |  |  |  |  |  |  |  |  |  |
|  | A31 |  | A32 |  | A33 |  |  |  |  |  |  |  |  |
|  | Select process per seam | ► | Look up WPS | ► | Assign weld parameters |  |  |  |  |  |  |  |  |
|  | Not started |  | Not started |  | Not started |  |  |  |  |  |  |  |  |
|  | ID / Sub-function | Description |  | Input |  | Control |  | Output |  | Mechanism | Owner | Status ▼ | Phase |
|  | A31  Select process per seam | MIG or laser depending on material, thickness, joint and access. |  | D04 Seam list |  | Process selection rules |  | Process per seam |  | Process / robot engineer | TBD | Not started | Phase 1 |
|  | A32  Look up WPS | Find the WPS for material / thickness / joint / position. |  | Process per seam |  | D05 WPS library |  | WPS reference |  | Process / robot engineer | TBD | Not started | Phase 1 |
|  | A33  Assign weld parameters | Current, voltage, wire feed speed, laser power, travel speed, gas flow, weave. |  | WPS reference D04 Seam list |  | D05 WPS library |  | D06 Seam process parameters |  | Process / robot engineer | TBD | Not started | Phase 1 |
