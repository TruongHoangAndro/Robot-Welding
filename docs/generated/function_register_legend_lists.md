# Function Register — sheet "Legend & Lists"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.1.xlsx` (read-only source, values only). Do not edit; re-export instead.

| c1 | c2 | c3 | c4 | c5 | c6 | c7 |
|---|---|---|---|---|---|---|
|  | Legend & Lists |  |  |  |  |  |
|  | Doc. No. APLSMN601-DES-003  /  Rev 0.1  /  2026-10-12  /  Prepared by: Truong Hoang  /  Status: Draft |  |  |  |  |  |
|  | How to use |  |  |  |  |  |
|  | 1 | 'A0 Overview' shows the top-level functions A1-A9. Click a block (or a row in the table) to open its sheet. |  |  |  |  |
|  | 2 | Each function sheet (A1...A9) shows its sub-functions as blocks and an IDEF0 table. Click a sub-function block to jump to its row. |  |  |  |  |
|  | 3 | 'Function Register' holds the text, owner and phase of every function - edit them there. STATUS of a sub-function is selected on its sheet A1...A9 (column M, drop-down); the Register and Overview read it automatically. |  |  |  |  |
|  | 4 | To add a sub-function: add a row in the Function Register with the next ID (e.g. A15, parent A1, level 2). It appears automatically (max 9 per parent, first 5 as blocks). |  |  |  |  |
|  | 5 | Status of a level-1 function is calculated from its sub-functions (all Done = Done; any started = In progress). |  |  |  |  |
|  | 6 | 'Data Items' lists every data / material that flows between functions. Check column flags items with no producer or no consumer. |  |  |  |  |
|  | IDEF0 (ICOM) |  |  |  |  |  |
|  | Input | What the function transforms (data or material), e.g. D01 STEP file. |  |  |  |  |
|  | Control | What constrains or guides the function, e.g. WPS, drawings, safety rules, standards. |  |  |  |  |
|  | Output | What the function produces, e.g. D04 Seam list. An output of one function is the input or control of another. |  |  |  |  |
|  | Mechanism | Who or what performs the function: software, robot, PLC, welder, operator, engineer. |  |  |  |  |
|  | Numbering | A0 = whole system; A1...A9 = top-level functions; A11...A19 = sub-functions of A1, etc. Data items D01, D02 ... |  |  |  |  |
|  | Block colours | Grey = Not started, yellow = In progress, green = Done, red = On hold. |  |  |  |  |
|  | Lists (used by drop-downs) |  |  |  |  |  |
|  |  |  |  | Status | Phase | Level |
|  |  |  |  | Not started | Phase 1 | 1.0 |
|  |  |  |  | In progress | Phase 2 | 2.0 |
|  |  |  |  | Done |  | 3.0 |
|  |  |  |  | On hold |  |  |
