# Document review before coding — 2026-10-29

Reviewer: software agent, for Truong Hoang. Status: findings resolved in **draft** documents, not approved.

| Reviewed | Revision |
|---|---|
| DES-001 Software Architecture | Rev 0.4 (2026-10-22) |
| DES-003 Function Register (IDEF0) | Rev 0.4 (2026-10-22), all 15 sheets |
| TST-002 Collision Avoidance Spike | Rev 0.2 (2026-10-22) |
| Change proposal CP-9…CP-23, CLAUDE.md, PLAN.md | 2026-10-27 |
| Cell CAD `docs/3D/APLSMN601_Automated Weldment System.step` | Fusion export 2026-10-27 |

Method: manual reading plus a script that checks every IDEF0 flow — each data item (D01…D21) and each internal
flow label must have a producer (or an external source) and a user; a level-1 function's ICOM must equal the union
of its sub-functions' ICOM. The same check passes on the Rev 0.5 draft (only external inputs and Phase 2 / test-tool
outputs remain open, which is correct).

## Results

| Output | File |
|---|---|
| DES-001 Rev 0.5 draft | `docs/generated/drafts/APLSMN601-DES-001_Software_Architecture_Rev0.5_DRAFT.md` |
| DES-003 Rev 0.5 draft | `docs/generated/drafts/APLSMN601-DES-003_Function_Register_Rev0.5_DRAFT.xlsx` (Markdown export in `drafts/function_register_rev0.5/`) |
| Build scripts (reproducible, exact replacements from Rev 0.4) | `docs/generated/drafts/make_des001.py`, `make_des003.py` |

Adoption: review the drafts → save them on Drive as Rev 0.5 masters → copy them unchanged into `docs/reference/`.

## A. IDEF0 structure (DES-003)

| # | Finding in Rev 0.4 | Resolution in the Rev 0.5 draft |
|---|---|---|
| I-01 | D14 Cell model has no producer ("External (design)"), but it is built by software | New **A18 Import cell CAD** produces D14 from the new external item **D21 Cell & fixture CAD** |
| I-02 | D08 produced only by A53 / A55; A51 "TCP data" and A52 "Fixture frame" go nowhere | A51 / A52 output D08 (TCP / fixture frame); A18 outputs the nominal D08 |
| I-03 | D07 is written by A45 **before** timing (A47) and limit checks (A48); A48 output "Joint path (limit-checked)" is unused | Chain A44 → A45 (collision-checked joint path) → A47 (timed) → **A48 writes D07** |
| I-04 | "Collision list" (A45) unused | Input of A48; stored as per-point status in D07, used by A71 (red points) |
| I-05 | "Error budget report" (A54) unused | Input of A74 / A7 (production release gate) |
| I-06 | "Pairs (tool pose, positioner angle)" (A49) unused | "Positioner angles" → input of A44 |
| I-07 | "Collision mesh" (A13) unused | "Part collision mesh" → input of A15 (part of D19) |
| I-08 | D19 used only by A44; A45 and A71 check collisions without the scene | D19 input of A45, A49; control of A7 / A71 |
| I-09 | A43 plans air moves before A44 has chosen joint states | A43: approach / retract poses before A44; air moves after A44 / A45 (feedback "Joint path") |
| I-10 | A81 (job selection) does not take D10 / D08, though the release and calibration hash check happens there (DES-001 6.1) | D10, D08 added to A81 |
| I-11 | "Alarm / stop" (A84) unused | Input of A85 |
| I-12 | D03 used only by A64 | Also A85 (log per part) and A8 |
| I-13 | "User accounts & permissions" (A88) unused | Control of A2 / A21, A74, A81 |
| I-14 | Level-1 ICOM ≠ union of sub-functions: A1 (I, C, O), A2 (I), A3 (C, O), A4 (I, C), A6 (I), A7 (I, C), A8 (I, C, O), A9 (I) | Aligned (script check passes) |
| I-15 | Function Register row 5 "Web UI - Later" without ID, although the Web UI is Phase 1 (ADR-09) | Removed |
| I-16 | "Edit in Function Register ►" links on sheets A1…A9 point to wrong rows (A1 → placeholder row 5, A9 → row of A73) | Recomputed from the row of each function |
| I-17 | Formulas (status, Data Items counts / Check) have no cached values: any values-only export shows them empty | `fullCalcOnLoad` set; the Check logic was verified by script (all D items OK) |
| I-18 | A0 Overview: stray text "Swap to A2."; "D08 → A6, A8" misses A4 | Removed; "D08 → A4, A6, A8" |
| I-19 | Sheet "A0 Overview Phase 1" reuses IDs A1–A4 with another meaning, STL input, pose "X,Y,Z,Q1,Q2,Q3" (3 quaternion terms are not a full rotation) | Renamed to work packages **P1–P4** mapped to functions, STEP input, pose = position mm + quaternion (x, y, z, w); owners and e-mail links kept |
| I-20 | Owner = TBD on all 60 functions | **Open** — proposal in section F (OI-28) |

## B. DES-003 content vs. the decisions of 2026-10-27

| # | Finding | Resolution |
|---|---|---|
| C-01 | A21 / A88: operator "Seam setup" permission | Engineer only (CP-21) |
| C-02 | A22: "tack welds first" (robot) | Tacks by hand after clamping, A81 (CP-19) |
| C-03 | A41: "Z … out of the joint" vs TST-002 "Z = −bisector" | Z out of the material (up), beam axis = −Z; tube butt joint: Z = surface normal (CP-9) |
| C-04 | A42 / A51: TCP "focus point / wire tip" vs TST-002 "10 mm beyond the nozzle" | TCP at the nozzle / wire tip, focus offset default 0, only the nozzle tip may touch (CP-10) |
| C-05 | A47, A49, A55, D14: positioner = Phase 2 | Phase 1, ROS 2 control, Modes C / B (CP-20); A49 Sim-1, A55 Stub |
| C-06 | A48 ±270° vs TST-002 C3 ±180° | Two checks: ±270° absolute, ±180° within a seam |
| C-07 | A74 "Not in sim", but Sim-2 replays a "released" trajectory | Stub: simulation-scope release (CP-17, ADR-17) |
| C-08 | D09 / D10 format "URScript / ROS2 trajectory" vs DES-001 JSON | JSON joint-trajectory segments + events; URScript fallback |
| C-09 | A4 / A44 do not use D08, although planning follows calibration (CP-18) | D08 input of A4 / A44 |
| C-10 | A15 single step; no placement study | A15 = placement with source `study` / `fixture_cad`; new **A17 Study part placement** (CP-22) |
| C-11 | A62, A63, A82, A83, A84: no positioner | Positioner segments, events only with robot **and** positioner at rest, drive ready / fault, drive fault alarm |
| C-12 | Revision History decision 1 (positioner Phase 2) | New decisions table 2026-10-27 (10 rows) supersedes it |

## C. DES-001

| # | Finding in Rev 0.4 | Resolution in the Rev 0.5 draft |
|---|---|---|
| E-01 | §1.2, AD9, ADR-06, ADR-11, §6.3: positioner as Phase 2 slot, part on a fixed fixture | Positioner Phase 1 under ROS 2, Modes B / C, PLC-driven option rejected with reason, safety in hardware (STO, drive fault → PLC) |
| E-02 | §1.3: R3 = DES-003 Rev 0.2, R10 = TST-002 Rev 0.1; cell CAD not referenced | R3 Rev 0.5, R10 Rev 0.2; new R12 (cell CAD), R13 (change proposal) |
| E-03 | §1.4: "D01…D19 of Function Register Rev 0.2" | D01…D21, Rev 0.5; new terms Mode B / C, marker component, release scope |
| E-04 | §4.2 / §8 / §12: `sim` = URSim, CLAUDE.md `sim` = mock | `sim_mock` (Sim-1), `sim_ursim` (Sim-2) |
| E-05 | §4.3: "reuses … pick_ik setup" contradicts ADR-12 | Analytic UR IK, 8 branches; pick_ik not used for the sweep |
| E-06 | §4.3: Z wording, no joint-step bound, one wrist-3 check, no dense check | CP-9 wording, ≤ 5° per step in one branch, two wrist-3 checks, dense check ≤ 0.02 rad |
| E-07 | §5.1: D07 poses only; D10 without scope; D14 producer "Design"; D19 without source | D07 = poses + timed joint path + positioner + status + D08 hash; D10 scope; D14 from A18; D19 source; new D21 |
| E-08 | §5.2: no `cache/model.brep`, `collision.stl`, no place for the cell CAD | `cell/cad/`, `cell/model/`, `job/fixture.step`, cache files; rule "production release needs `fixture_cad`" |
| E-09 | §5.3: frame chain without the positioner; `tcp_laser` = focus point; no axis convention; nominal frames source unclear | `base_T_tcp = base_T_axis · Rz(θ) · flange_T_fixture · fixture_T_part · part_T_pose · tcp⁻¹`; TCP at the nozzle tip; Z-up convention; frames from `REF_*` markers (incl. UR `base` vs `base_link`) |
| E-10 | §5.4: ACM "TCP link"; three clearance values (3 / 5 / 10 mm) with no stated purpose; head source "vendor STEP" | `nozzle_tip` contact only; `padding_mm` / `weld_clearance_mm` / `air_clearance_mm` defined; head and holders from the Fusion CAD |
| E-11 | §6.1 / §6.2: no positioner homing, no manual load / clamp / tack step, no Mode C sequence, speed scaling only for the robot | INIT homes the positioner; READY includes manual tack; Mode C / B sequence; 100 % rule for positioner weld segments |
| E-12 | §7.1 / §7.2 / §8: no positioner interface; IPC has 2 ports | Positioner controller action; EtherCAT drive link, drive ↔ PLC / safety hard-wired; IPC 3 × Ethernet |
| E-13 | §7.3 / §7.4: "Seam setup"; no placement or cell-model endpoints; release without scope | Engineer-only seams; `/jobs/{id}/placement`, `/cell/model`; release scope |
| E-14 | ADR-15: pythonocc 7.9 / Python 3.12 | "≥ 7.9; tested with 8.0.1 / Python 3.10" |
| E-15 | No ADR for the CAD source and for release scopes | New **ADR-16** (Fusion STEP + markers), **ADR-17** (release scopes) |
| E-16 | §11: no A17 / A18; A49 / A55 Phase 2 | A17 R1 / R3, A18 R1, A49 R1, A55 R2 |
| E-17 | §13: OI-13, OI-20 open; OI-14, OI-15, OI-17 outdated | OI-13 / OI-20 closed; OI-14 / 15 / 17 updated; **new OI-22…OI-28** |
| E-18 | Figures 1, 2, 3, 5, 7 outdated (OI-17) | **Open** — drawings not changed; positioner added to OI-17 |
| E-19 | §11 release names still "First MIG Weld" (charter CR pending) | **Open** — PM item, unchanged |

## D. TST-002 — changes needed for Rev 0.3 (not drafted: Rev 0.3 is the results revision)

| # | Change |
|---|---|
| T-01 | §4 D1 "Z = −bisector" → "beam axis = −Z; seam frame Z = bisector out of the material" (CP-9) |
| T-02 | §3.1 TCP "10 mm beyond the nozzle tip" → at the nozzle / wire tip; C2 "≥ 5 mm" applies to head body, mount, conduit (CP-10) |
| T-03 | §3 inputs I1–I4 now come from the cell CAD (golden part 3-part assembly, head and mount, jig and holders, robot base) — status "available, nominal"; head is a surface model |
| T-04 | Scope: seams J1 (Mode C) and J2 (robot-only, then Mode B) instead of "1 most constrained seam"; scene includes the positioner joint |
| T-05 | C8: report coverage robot-only **and** with the positioner; decision rule "input to OI-20" → OI-20 is closed |
| T-06 | E1: pythonocc-core 8.0.1 / Python 3.10 tested |
| T-07 | **Schedule:** today is D5 (29 Oct). The spike has not been run in this workspace; the Go / Pivot decision needs a new date |

## E. Workspace documents

| # | Item | Action |
|---|---|---|
| W-01 | CLAUDE.md reads DES-001 Rev 0.4 first | Pointer to the Rev 0.5 drafts added; switch the @-include after approval |
| W-02 | PLAN.md uses the Rev 0.4 function list | A17 / A18 named in T0.2 / T2.4; TST-002 date flagged |
| W-03 | Wiki (read-only in coding sessions) | For the next wiki session: L-001 resolved by CP-10; L-003 is wrong (units were mm); L-005 launch names; L-007 wrist-3 two checks; L-012 sheet P1–P4; overview milestones and decisions of 2026-10-27 |
| W-04 | DES-002 (ICD, source of D11) does not exist yet | Unchanged; needed before WP4 / hardware |

## F. Decisions needed from Truong

1. **Approve the drafts** (or list changes), then publish Rev 0.5 on Drive.
2. **Function owners (OI-28).** Proposal from the team sheet P1–P4:

   | Functions | Proposed owner |
   |---|---|
   | A1, A2 (P1) | Nguyen Huu Thang, Truong Hoang |
   | A4, A6 (P2) | Thanh Luu, Khang Doan |
   | A7 (P3) | Thanh Luu, Truong Hoang, Nguyen Huu Thang |
   | A8 (P4) | Thanh Luu, Khang Doan |
   | A3 (LightWELD presets, WPS) | Khang Doan + welding engineer |
   | A5 (calibration), A9 (inspection) | TBD |

3. **J2 strategy (OI-27):** robot-only first, Mode B only if robot-only < 100 % (proposed).
4. **New date for the TST-002 decision** (T-07).
5. **Who redraws the DES-001 figures** (OI-17).

## G. Decisions of 2026-10-29 (Truong Hoang)

1. Rev 0.5 accepted; release files in `docs/generated/release_rev0.5/` (produced by `drafts/finalize_rev05.py`), to be
   stored on Drive as masters.
2. Owners: all functions by the software agent (Claude Code); Truong Hoang gives direction and runs the simulation
   tests. Execution on hardware assigned later (OI-28 closed).
3. J2: robot-only first; positioner Mode B only if robot-only coverage < 100 % (OI-27).
4. Still open: new date for the TST-002 decision (T-07), figure redraw (OI-17).
