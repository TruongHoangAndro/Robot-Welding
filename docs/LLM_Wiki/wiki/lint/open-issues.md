---
id: open-issues
title: Open issues (lint queue)
type: lint
domains: [PM, WELD, COLL, UR5E, SYNC]
tags: []
status: open
updated: 2026-10-27
---

# Open issues

Queue of contradictions and drift found by ingest and lint. Seeded from the first full read (PROJECT.md Appendix A, 2026-10-27). Nothing has been changed in any source.

| L-# | Sev. | Check | Finding | Sources | Proposed fix | Status |
|---|---|---|---|---|---|---|
| L-001 | High | L4 / L6 | LightWELD fires only when the **nozzle touches the part** (part-contact circuit; robot must press the nozzle on the part). TST-002 §3.1 assumes TCP 10 mm beyond the nozzle tip, C2 requires nozzle clearance ≥ 5 mm and DES-001 ACM lets only the TCP link touch seam faces | LASER WELDING slides 6–7; TST-002 §3.1, §5; DES-001 §5.4; OI-13 | Resolve OI-13 before the TST-002 result is used: nozzle-contact allowed in ACM, TCP at contact point, need for compliance / force; record as decision | open |
| L-002 | Medium | L1 | DES-001 Rev 0.4 §1.3 cites WBS Rev 0.1, DES-003 Rev 0.2, REQ-002 "planned", TST-002 Rev 0.1 (current: 0.3 / 0.4 / Rev 0.1 exists / 0.2); §1.4 says D01–D19 of FR Rev 0.2, §5.1 says D01–D20 of Rev 0.3 | DES-001 Rev 0.4 | Update references in DES-001 Rev 0.5 | open |
| L-003 | Medium | L3 | DES-003 revision history Rev 0.3 says "units = m"; ADR-14 (same date) says mm for all project data | DES-003 Rev 0.4 "Revision History"; DES-001 ADR-14 | Correct the note in DES-003 Rev 0.5 | open |
| L-004 | Medium | L4 / L10 | pythonocc-core 7.9 (ADR-15 with Python 3.12; TST-002 E1; WBS 3.2) vs 8.0.1 / Python 3.10 env `awm_occ` (workspace `CLAUDE.md`, already marked [TBC]) | DES-001 ADR-15; TST-002; WBS; `CLAUDE.md` | Decide one version; update ADR-15 | open |
| L-005 | Low | L4 | Launch `sim` = URSim in DES-001 §4.2 (`awm_bringup`); `sim` = mock hardware (Sim-1) and URSim separate in `CLAUDE.md` | DES-001 §4.2; `CLAUDE.md` | Align names (e.g. `sim_mock`, `sim_ursim`) | open |
| L-006 | Medium | L1 / L2 | Claude Code prompts v1 / v2 point to DES-001 Rev 0.2 and DES-003 Rev 0.1, WPS "MIG + laser", Qt GUI out of scope | `Claude outputs\APLSMN601_ClaudeCode_Prompt_*.md` | Mark superseded by `CLAUDE.md` 2026-10-25 or issue prompt v3 | open |
| L-007 | Medium | L4 | Wrist-3 limit: TST-002 C3 "±180° of start" vs README open points "wrist-3 ±270°" | TST-002 §5; README (working) | Choose one value per purpose (spike criterion vs soft limit) and record in parameters register | open |
| L-008 | Low | L4 | Wobble range: slides "up to 5 mm"; DES-001 OI-18 "0–15 mm (distributor) vs max 5 mm (team note)" | LASER WELDING slide 2, 4; DES-001 OI-18 | Confirm in IPG manual; close OI-18 | open |
| L-009 | Medium | L8 | Question List now has Apillis comments for Q1–Q4 (UR5e + Jazzy; Phase 1 simulation; path from CAD STEP / STL; set-up and calibration at initialisation + sensor to detect the part); README says "no answers yet". Q6 says "run simulation with **152056** parts" — no STEP or drawing 152056 in the project (golden part in TST-002 is 564082). Q3 mentions STL while ADR-02 is STEP B-Rep (STL = Phase 2 fallback A16) | Question List (Google Sheet); README; DES-001 ADR-02 | Confirm part number 152056 vs 564082; update README / Q&A status | open |
| L-010 | Medium | L2 | Still MIG-first: Charter Rev 0.2, Kickoff Rev 0.1, schedule xlsx (MIG in rows 3.5, 5.3, 6.3, 7.2, 7.3; automatic seam detection still in row 8.3), REQ-002 Rev 0.1 (MIG components and operations). No CR file in `2…\06. CHANGE REQUESTS\` (README only) | PM-001, PM-002, PM-004 xlsx, REQ-002; PM-004 v2 §5 | Charter CR (owner Truong, due 2026-10-26 per PM-004 v2); REQ-002 Rev 0.2 laser-focused | open |
| L-011 | Low | L1 | Schedule "Docs Tracking" sheet lists PM-003 Rev 0.1 and REQ-002 / DES-001 as "Planned" | PM_Schedule.xlsx | Update with the schedule change | open |
| L-012 | Medium | L5 | DES-003 sheet "A0 Overview Phase 1" uses A1–A4 for a 4-block team pipeline (input "File STL"; pose X, Y, Z, Q1, Q2, Q3 + cf1–cf3) — clashes with IDEF0 A1–A9, and Q1–Q3 is ambiguous (rotation vector or 3 of 4 quaternion terms) | DES-003 Rev 0.4 | Rename blocks (e.g. P1–P4), state pose convention (UR rotation vector Rx, Ry, Rz?) and units | open |
| L-013 | Low | L8 | WBS 4.6 dictionary says criteria C1–C7; TST-002 Rev 0.2 moved C7 to the hardware stage and added C8 | WBS Rev 0.3; TST-002 Rev 0.2 | Update in WBS Rev 0.4 | open |
| L-014 | Low | L10 | Master `README.md` (last check 2026-10-12) is behind the working README (2026-10-22) | `{MASTER}\README.md`; project README | Sync after review | open |
| L-015 | Low | L10 | TST-002 records folder `4…\4. SOFTWARE\5. Spike\` does not exist yet; collision material sits in `4. Collision\`. P2 demo v1 / v2 exist in both `Claude outputs\` and `4. Collision\` | Folder listing 2026-10-27 | Decide one location for spike records | open |
| L-016 | Low | L7 | "AI Solution" collision concept (Collision Solution slide 6) has no doc number and no link to DES-001 / ADR-12 | Collision Solution slides | Backlog page + research question | open |
| L-017 | Info | L8 | TST-002 timebox 23–29 Oct; results due in Rev 0.3 on 2026-10-29; Sim-0 milestone 2026-10-30 | TST-002 §4; PM-004 v2 | Track as tasks; ingest results when available | open |
