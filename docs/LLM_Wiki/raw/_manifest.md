---
id: _manifest
title: Raw source manifest
type: register
domains: [PM, SYNC]
tags: [pm/docs, sync/drive]
status: current
updated: 2026-10-27
---

# Raw source manifest

Every raw source the wiki knows about. Claude updates this table on each ingest. Dates = file modified date (Asia/Bangkok) seen on 2026-10-27.

## Root aliases

| Alias | Path | Note |
|---|---|---|
| `{MASTER}` | `G:\Shared drives\API-US-ENGINEERING\4. PRODUCTS\AUTOMATED WELDING MACHINE\` | Company master — read only for tests |
| `{WORK}` | `G:\My Drive\20_Customer\Automatic Welding\` | Working copy — edits as new Rev files |
| `{WS}` | WSL `$HOME/ros2/aplsmn601_ws` (Git) | ROS 2 workspace; `docs/reference/**` frozen |
| `{WIKI}` | `{WS}/docs/LLM_Wiki/` (same Git repo as the code) | This wiki |
| `{GDOC}` | Google Drive files (connector) | Read only |
| `{INBOX}` | `raw/inbox/` | New material from Truong |

## Sources

| # | Batch | Doc no. | Title | Rev | Location | Modified | Ingested | Source page |
|---|---|---|---|---|---|---|---|---|
| R001 | 1 | APLSMN601-DES-001 | Software Architecture Description | 0.4 (md) / 0.1 (docx) | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/APLSMN601-DES-001_Software_Architecture_Rev0.4.md` | 2026-10-22 | no | — |
| R002 | 1 | APLSMN601-DES-003 | Function Register (IDEF0 A0–A9, D01–D20, Sim stage) | 0.4 | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/3. Function Register/APLSMN601-DES-003_Function_Register_Rev0.4.xlsx` | 2026-10-22 | no | — |
| R003 | 1 | APLSMN601-TST-002 | Collision Avoidance Feasibility Spike | 0.2 | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/APLSMN601-TST-002_Collision_Avoidance_Spike_Rev0.2.md` | 2026-10-22 | no | — |
| R004 | 1 | — | ROS 2 workspace CLAUDE.md (aligned with DES-001/003 Rev 0.4, TST-002 Rev 0.2) | 2026-10-25 | `{MASTER}/Claude outputs/CLAUDE.md (copy of {WS}/CLAUDE.md)` | 2026-10-25 | no | — |
| R005 | 1 | — | Project README + document register (working folder) | 2026-10-22 | `{WORK}/README.md (also claude.ai Project doc claude/README.md)` | 2026-10-22 | no | — |
| R006 | 1 | — | Project README (master, older) | 2026-10-12 | `{MASTER}/README.md` | 2026-10-16 | no | — |
| R007 | 2 | APLSMN601-PM-001 | Project Charter (ENG.01.21) | 0.2 | `{MASTER}/2 SCHEDULE AND MINUTES/00. PROJECT CHARTER/APLSMN601-PM-001_Project_Charter.md (+ _Rev0.2.docx)` | 2026-10-12 | no | — |
| R008 | 2 | APLSMN601-PM-002 | Phase 1 Kickoff Presentation | 0.1 | `{MASTER}/2 SCHEDULE AND MINUTES/01. KICKOFF/APLSMN601-PM-002_Kickoff_Presentation.md (+ _Rev0.1.pptx)` | 2026-10-12 | no | — |
| R009 | 2 | APLSMN601-PM-003 | Work Breakdown Structure | 0.3 | `{MASTER}/2 SCHEDULE AND MINUTES/02. PLAN/APLSMN601-PM-003_WBS_Rev0.3.md (+ .xlsx)` | 2026-10-22 | no | — |
| R010 | 2 | APLSMN601-PM-004 | Schedule change proposal v2 (Sim-0/1/2) | v2 | `{MASTER}/2 SCHEDULE AND MINUTES/02. PLAN/APLSMN601-PM-004_Schedule_Change_Proposal_20261022_v2.md` | 2026-10-22 | no | — |
| R011 | 2 | APLSMN601-PM-004 | Project Schedule (company template) | Draft | `{MASTER}/2 SCHEDULE AND MINUTES/02. PLAN/APLSMN601-PM_Schedule.xlsx` | 2026-10-23 | no | — |
| R012 | 2 | APLSMN601-REQ-002 | Risk Assessment ISO 12100 (UCDAM00001) | 0.1 | `{MASTER}/3 RISK ASSESSMENT/APLSMN601-REQ-002_Risk_Assessment_Rev0.1.xlsx` | 2026-10-12 | no | — |
| R013 | 3 | — | LASER WELDING FOR AUTOMATION WELDING SYSTEM (LightWELD 1000) | Google Slides | `{GDOC} gfile_id 1aKfJ3dE3YipGqCduaDXKQCqAimzPZzljydzKQ9bDjsA (link in 12 DOCUMENTS)` | 2026-10-16 | no | — |
| R014 | 3 | — | IPG LightWELD brochure | PDF | `{MASTER}/12 DOCUMENTS/IPG_LightWELD_BR_EN_LTR.pdf` | 2026-10-13 | no | — |
| R015 | 3 | — | IPG LightWELD Cobot brochure | PDF | `{MASTER}/12 DOCUMENTS/IPG_LightWELD_Cobot_BR_EN_LTR.pdf` | 2026-10-13 | no | — |
| R016 | 3 | — | Collision Solution (pipeline, priorities, ladder graph, AI Solution idea) | v0.1 (Slides) | `{GDOC} gfile_id 1D4U3d8rrAv_25NNvaelT4pFvf2DUQaQxIm3Bg9G_qdE (link in 4. Collision)` | 2026-10-26 | no | — |
| R017 | 3 | — | P2 interactive demo (VI/EN) | v2 | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/4. Collision/APLSMN601_P2_Demo_v2.html` | 2026-10-23 | no | — |
| R018 | 4 | APLSMN601-DES-004 | Function Register As-Built (Tac demo, F01–F15) | 0.1 | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/3. Function Register/APLSMN601-DES-004_Function_Register_AsBuilt_TacDemo_Rev0.1.xlsx` | 2026-10-14 | no | — |
| R019 | 4 | — | Tac demo ROS 2 workspaces (4 zips) | — | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/Codes/*.zip (duplicate in From Tac Huynh/)` | 2026-10-05 | no | — |
| R020 | 4 | — | Claude outputs practice package (motion_server, sim_arm, welder_node, msgs/srvs/actions) | — | `{MASTER}/Claude outputs/*.py, *.msg, *.srv, *.action` | 2026-10-20 | no | — |
| R021 | 4 | — | Claude Code prompts (A1–A7 v1, v2) — superseded by CLAUDE.md? | v1, v2 | `{MASTER}/Claude outputs/APLSMN601_ClaudeCode_Prompt_*.md` | 2026-10-19 | no | — |
| R022 | 4 | — | Architecture team review deck | 0.1 | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/4. SOFTWARE/APLSMN601-DES-001_Architecture_Team_Review_Rev0.1.pptx` | 2026-10-17 | no | — |
| R023 | 5 | — | Question List (Q&A 1–6) | Google Sheet | `{GDOC} gfile_id 1uomW5qXuXMvM-9sSzBLj-jcFYOijby5xT6Q5xtSF7p4 (link in 6. Q&A)` | 2026-10-14 | no | — |
| R024 | 5 | — | Customer drawings 111373, 564049, 564082, 564085, 564103 + 13 photos | PDF/JPG | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/3. MECHANICAL/CustomerDrawings/` | 2026-10-05 | no | — |
| R025 | 5 | — | STEP models (8): 177345, 296783, 300812, 564082 ×3, 564141 ×2 | STEP | `{MASTER}/4 DRAWINGS AND DESIGN EVALUATION/3. MECHANICAL/StepFiles/` | 2026-10-14 | no | — |
| R026 | 5 | — | Meeting 2026-10-13 recording (needs transcript) | mp4 | `{MASTER}/2 SCHEDULE AND MINUTES/03. MINUTES/Meeting For Automated Welding Machine Project - 2026_10_13 ….mp4` | 2026-10-14 | no | — |
| R027 | 5 | — | UR5e_Welding_Robot_Project_Tracker (P0–P3, W0–W9, Q1–Q7, R1–R9) | Google Sheet | `{GDOC} gfile_id 1F_eKd6XCSp9NB2t3hE8qv75ARI5GfE8VastQ1UajlDQ` | 2026-10-22 | no | — |
| R028 | 5 | — | Automated Weldment Design Review (outline) | Google Slides | `{GDOC} gfile_id 1dPzDJaBtoNP_ZQP2vyzzTDwWp5NwSxMymKRI97qFVS8 (master root)` | 2026-10-10 | no | — |
| R029 | 5 | — | Automated Weldment System_Design Review_Summary (outline) | Google Slides | `{GDOC} gfile_id 1-ukSCgkj4eJagFR6vn0sgw1tTQP724zispS5X1cByNQ (5. DFM)` | 2026-10-13 | no | — |
| R030 | 5 | — | Automatic Weldments Logs (tabs only, empty) | Google Doc | `{GDOC} gfile_id 1l2cMJL1_HaioTTsSFBNoLrBLlC25zTI5AkcIwK0C5m0 (6. Q&A)` | 2026-10-09 | no | — |

Batch = step of PROJECT.md Section 13 (1 core design · 2 PM · 3 welding & collision · 4 code · 5 customer & parts).
