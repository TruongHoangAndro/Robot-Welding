# Function Register — sheet "Revision History"

Exported from `docs/reference/APLSMN601-DES-003_Function_Register_Rev0.4.xlsx` (values only). Do not edit; re-export with `scripts/export_function_register.py`.

| c1 | c2 | c3 | c4 |
|---|---|---|---|
| Revision History |  |  |  |
| Doc. No. APLSMN601-DES-003  \|  Rev 0.4  \|  2026-10-22  \|  Prepared by: Truong Hoang  \|  Status: Draft |  |  |  |
| Rev | Date | Description | By |
| 0.1 | 2026-10-12 | First draft: IDEF0 decomposition A0-A9, sub-functions, data items D01-D15. | Truong Hoang |
| 0.2 | 2026-10-22 | Gap analysis against UR5e_Welding_Robot_Project_Tracker (roadmap 15 Oct 2026). New sub-functions: A15, A16, A25, A34, A46, A47, A48, A49, A54, A55, A86, A87, A95. Descriptions updated: A1, A11-A13, A21-A24, A33, A4, A41-A45, A5, A51-A53, A62, A63, A71-A73, A81, A83-A85, A92. New data items D16-D19. Placeholder row "Paramter collision / Collison list" merged into A45 (output Collision list). Phase 2 slots for positioner (A49, A55) per decision 2026-10-22. Column M (Notes) gives the Tracker source of each change. | Truong Hoang |
| 0.3 | 2026-10-22 | Second review 2026-10-22: Phase 1 process = laser only (IPG LightWELD 1000, PLC-controlled), MIG to Phase 2; A23 automatic seam detection deleted (seams picked in the 3D simulator view, once per part number); A25 kept as test tool; units = mm for all project data; Web UI decided (A13 exports D20 view model, new A88 users & roles); A31, A33, A34, A42, A63, A82-A86 rewritten for the LightWELD; A49 sync options added. New data item D20. | Truong Hoang |
| 0.4 | 2026-10-22 | Simulation stage focus: new columns N "Sim stage" (Sim-1 MoveIt mock / Sim-2 URSim / Stub / Not in sim) and O "Sim note" in the Function Register; legend added. No function content changed. | Truong Hoang |
| Decisions taken 2026-10-22 (Rev 0.2 and Rev 0.3) |  |  |  |
| # | Topic | Decision | Ref |
| 1 | Positioner | Phase 1 keeps fixed fixture on the Pismo frame (UR5e only). Positioner functions A49, A55 kept as Phase 2 slots. | DES-001 ADR-06, ADR-11 |
| 2 | Welder control | PLC owns the IPG LightWELD 1000 (trigger, preset, interlocks, wire feeder). Phase 1 = laser only; MIG in Phase 2. | DES-001 ADR-04 |
| 3 | UI technology | Decided 2026-10-22: Web UI (React + Three.js + FastAPI); seam picking in 3D (DES-001 4.5, 7.4). | DES-001 ADR-09 |
| 4 | ROS distro | Ubuntu 24.04 + ROS 2 Jazzy for Phase 1 (Docker image of the Tac demo, used in TST-002). | DES-001 ADR-10 |
| 5 | Seam detection / planner | Automatic seam detection removed from scope; seams picked manually once per part number. MoveIt2 + graph search (TST-002), Tesseract as fallback. | DES-001 ADR-03, ADR-12 |
| 6 | Units | mm for all project data; third-party ROS packages stay in metres behind one adapter. | DES-001 ADR-14 |
