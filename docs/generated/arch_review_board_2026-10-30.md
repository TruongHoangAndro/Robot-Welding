# APLSMN601 — Architecture Review Board, 2026-10-30 (record)

| Field | Value |
|---|---|
| Subject | DES-001 Rev 0.6 draft (CP-24…CP-35) and the Rev 0.2 code base, as the basis for Sim-1, Sim-2 and the hardware stage |
| Board | Three AI reviewer agents and an AI chair, run by the software agent (Claude Code) |
| Record written | 2026-11-02 by the software agent (technical editor) |
| Status | **Input for the human review by Truong Hoang and Linh Tran — not a decision** |
| Applied to | `docs/generated/drafts/make_des001_rev06.py` → DES-001 Rev 0.6 draft; `docs/generated/DES-001_change_proposal.md` (section "Architecture Review Board 2026-10-30", CP-36…CP-51) |

> **Important.** The board members are AI reviewer agents. They are roles with a review lens, not people, and they
> carry no authority. Everything below is a recommendation. Decisions are taken only by the named humans (Truong
> Hoang, Linh Tran, Tac Huynh). Escalations E-01…E-08 are open items OI-37…OI-44 in the draft.

## 1. Purpose and inputs

Purpose: test whether DES-001 Rev 0.6 (draft) is a sound basis for Sim-1 (2026-11-20), Sim-2 (2026-12-04) and the
hardware stage, and propose the simplest changes that make it one.

Inputs:
- DES-001 Rev 0.5 (working baseline) and the Rev 0.6 draft with CP-24…CP-35;
- DES-003 Rev 0.5 function register (exports in `docs/generated/function_register_*.md`);
- TST-002 Rev 0.2 (collision-avoidance spike, criteria C1–C8);
- PLAN.md, PROGRESS.md, FINAL_REPORT.md, env_report.md, doc review of 2026-10-29;
- the Rev 0.2 code in `src/` (about 4,600 lines);
- public sources cited by the members (UR, ros2_controllers, MoveIt, IPG, ISO / IEC, laser-welding literature).

## 2. Members

| Member (AI role) | Lens |
|---|---|
| CTO / Chief Software Architect | Module structure, planner design, data contracts, build vs buy, code that can be kept |
| VP Quality, Functional Safety & Manufacturing Engineering | Verification evidence, laser and machine safety, release integrity, weld quality records, cybersecurity |
| Program / Delivery Director | Critical path, stage scope versus capacity, decision governance, external dependencies |
| Chair (AI) | Verifies key facts in the files, merges the positions, writes the consensus |

None of the members is a person. The software agent ran them, and the software agent also wrote the documents they
reviewed; the human review is the independent check.

## 3. Method

1. **Round 1 — independent review.** Each member read the inputs and wrote a verdict, a score, strengths, numbered
   proposals (CTO-01…15, QS-01…12, DEL-01…14) and a position on CP-24…CP-35.
2. **Round 2 — cross-examination.** Each member critiqued every proposal of the other two, listed conflicts,
   conceded or merged own proposals and added new ones (CTO-N01…N05, QS-N01…N05, DEL-N01…N05).
3. **Consensus by the chair.** The chair verified the decisive facts in the files (for example `sampling.py:91`,
   `release.py:91`, `WeldEvent.action`, Rev 0.6 l.336 / l.376), merged duplicates and wrote 25 decisions
   (B-01…B-25), a CP review and 8 escalations. Decisions that change a decision already taken by a human are
   escalated, not decided.
4. **Application by the editor.** The software agent applied the decisions to the Rev 0.6 build script, rebuilt
   the draft from Rev 0.5, checked tables / ids / section numbers, and appended the change-proposal section.

## 4. Verdicts and scores

| Member | Round 1 | Score R1 | Revised (round 2) | Score R2 |
|---|---|---|---|---|
| CTO / Chief Software Architect | approve with conditions | 6 | approve with conditions | 5 |
| VP Quality, Functional Safety & Manufacturing Engineering | approve with conditions | 6 | approve with conditions | 5 |
| Program / Delivery Director | approve with conditions | 5 | approve with conditions | 5 |
| **Board** | — | — | **approve with conditions** | **5 / 10** |

Board condition: the P0 decisions are written into DES-001 Rev 0.6, TST-002 Rev 0.3 and PLAN.md before spike code is
written. The TST-002 decision is planned for Fri 2026-11-06.

**What stays (all members).** Offline / online split; released = executed; hash-chained job package with atomic
writes; OCCT as a separate process; 100 %-or-reject; PLC owns the laser; weld events only at rest; Modes B / C
without synchronised motion; IPC software not safety-related.

**What the board found (verified by the chair).**
1. The spike inputs are not valid yet: seam normals taken per solid (about 45° error on J1 / J2); 2 mm pitch against
   the 5° continuity bound fails C3 by construction on Ø 26.67 mm tubes; clearance undefined (padded or not); blanket
   ACM exemption of `nozzle_tip` against the whole part; ψ treated as free although filler wire constrains it.
2. The release record is one job-level file that is overwritten; a manifest over editable files breaks the running
   release.
3. Rev 0.6 adds machinery Phase 1 does not need: gateway / bridge split, run queue, migration scripts, a byte-equal
   D07 target that cannot pass, a full observability stack.
4. Runtime contract gaps: MIG-typed WeldEvent, automatic emission retry, D09 with positions only, no stage after
   Sim-2, no release scope for A72 / A73 through M8 / M9, a PLC program "owned" by an agent that must not write it.

## 5. Key debates

| Topic | Positions | Resolution |
|---|---|---|
| Validity of the 2026-11-06 TST-002 decision | DEL: date it, incomplete evidence = Pivot signal. CTO: inputs wrong, slip rather than decide. QS: freeze contact model and ψ window too | Date kept; inputs frozen (B-01…B-05); missing tooling = slip ≤ 3 working days, not Pivot |
| Planner backend on the critical path | CTO: batched in-process backend P0. DEL: `moveit_py` not installed, C5-only failure = Go. QS: only distances matter, common-mode risk | Interfaces P0; first backend = `move_group` services with unpadded clearance; batched backend only if C5 > 30 s; independent M7 re-check (B-06) |
| Release integrity | CTO / QS: frozen bundle, full runtime check. DEL: runtime checks D10 → D09 / D15 / records / D08 only | Frozen bundle, one pointer per scope, runtime checks frozen manifest + context record; DEL conceded (B-09, B-10) |
| Planning-run transport, `awm_api` split | Rev 0.6: gateway + bridge + queue. CTO: ROS 2 action worker. DEL: CLI subprocess runner. QS: global lock, no cancel | ADR-21 rejected, queue removed; CLI runner, one slot, SIGTERM cancel; action worker decided at Sim-1 exit on measured scene build (B-11) |
| Deferral vs interfaces in Sim-2 code | DEL: defer hardware CPs as a block. QS / CTO: M8 / M9 / M10 sim are coded in Sim-2, so contract items must be fixed first | Interface fields, scope table, heartbeat, no retry, mixed-mode refusal, fault catalogue, weld-event table before WP4; hardware decisions to dated gates (B-14, B-17, B-19) |
| Positioner drive mode (ADR-22) | CTO: cyclic set-points only. QS: drive profile (withdrawn). DEL: hardware choice | Conformance wording: commands derived from D09, same generator in mock and M7, encoder tracking checked, hard-wired in-motion / fault; decide by 2026-11-20 (B-20) |
| Nozzle contact and ACM | Rev 0.6 / CTO: ACM `nozzle_tip` ↔ part. QS: hides penetration anywhere, accepts impossible tilted contacts | No ACM exemption; tip geometry stops at a standoff; explicit contact check by `face_id`; contact-zone poses in C1 (B-04) |
| Automatic emission retry | Rev 0.6, A63, DEL script: one retry. QS / CTO: double exposure risk | Retry deleted; timeout → EmissionOff confirmed → FAULT (B-17, B-19) |
| Recovery after FAULT | QS: RECOVERY with reverse replay. CTO: reverse segments must be released. DEL: Phase 2 with A86 | Manual recovery with the pendant, home / start-state check, NCR draft; no jog; reverse replay Phase 2 (B-13, B-19) |
| Accuracy decision timing | QS: decide the touch check now. CTO: interface now, method at HW-1. DEL: decide at HW-1 | Budget restructure and typed LOCATE_PART now; method at the HW-1 gate; no runtime per-part correction without E-07 (B-21) |
| Build image and environment | CTO: public-base image now. DEL: self-hosted runner on existing image. QS: hardened runtime image | Sim stages: existing image + self-hosted runner on a company remote; hardened image as HW-1 entry item, escalated (B-23, E-06) |

## 6. Consensus decisions

| B-id | Title | Decision | Prio | CP | Main DES-001 change |
|---|---|---|---|---|---|
| B-01 | TST-002 decision 2026-11-06 with frozen inputs | accept_modified | P0 | CP-36 | ADR-12 status, OI-29 |
| B-02 | Joint-aware seam frames between separate solids | accept_modified | P0 | CP-37 | M1 / M2 rows, 4.3 step 1, topology.json, D04, S2 |
| B-03 | Planner state space and acceptance metrics | accept_modified | P0 | CP-38 | 4.3 steps 1, 3–6, 5.4 clearance |
| B-04 | Explicit nozzle contact check, no ACM exemption | accept_modified | P0 | CP-39 | 5.4 Part / head rows, clearance, OI-22 |
| B-05 | ψ window per WPS | accept_modified | P0 | CP-40 | 1.4, 4.3 steps 2–3, D05, 12.2 E13, ADR-12 |
| B-06 | M4 core behind backends, M7 re-check, dependency rule | accept_modified | P0 | CP-41 | 4.1, 4.2 (awm_motion), 4.3, D15 |
| B-07 | D14 / D19 boundary, scene manifest, default-deny mapping | accept_modified | P0 | CP-42 | awm_description row, D14, D19, 4.5, ADR-14, ADR-16, 12.3 |
| B-08 | Migration order, D16 and D08 split | accept_modified | P0 | CP-43 | M0, D08, D16, 5.2 tree, OI-34 |
| B-09 | Immutable release bundle, pointer per scope | accept_modified | P0 | CP-29 | 5.2, D10, ADR-20, 4.4 |
| B-10 | One context record | accept | P0 | CP-44 | 5.2 rule, 5.3, 4.4, D07 / D09 / D10 |
| B-11 | CLI job runner, ADR-21 rejected | accept_modified | P0 | CP-28 | UI backend row, 4.3, ADR-19 / 21, 7.4, 12 |
| B-12 | Ordered Sim-1 slice | accept_modified | P0 | CP-34 | Principle 9, 4.3 step 3, D19 `verified`, 11 |
| B-13 | Staged Web UI, contract freeze 2026-11-04 | accept_modified | P0 | CP-32 | 4.5 Stage, 7.3, 7.4, 3 |
| B-14 | Delivery stages, revision per gate | accept_modified | P0 | CP-45 | New 2.3, 1.2, 9 Decide by, 11 Stage, 13 columns |
| B-15 | D09 = exact commanded trajectory | accept | P1 | CP-46 | D09, M6 / M7 / M9, 6.2, 12 |
| B-16 | `trial` release scope | **escalate (E-02)** | P1 | CP-47 | ADR-17 Proposed amendment, 6.2, D10 |
| B-17 | Typed interfaces v1, emission tied to axis motion, no retry | accept_modified | P1 | CP-27 | 6.1, 6.2, 7.1, 7.2 |
| B-18 | Named humans for PLC / safety / hardware; weld-event table | **escalate (E-03)** | P1 | CP-35 | OI-28 Proposed exceptions, M10, PLC row, 12.1, 12.3, ADR-23 |
| B-19 | Minimal Sim-2 runtime | accept_modified | P1 | CP-48 | ADR-08, principle 7, 6.1, M7, D15, 12.3 |
| B-20 | ADR-22 conformance wording | accept_modified | P1 | CP-33 | ADR-22, 6.2, 6.3, 7.2, OI-14 |
| B-21 | Accuracy chain restructured | accept_modified | P1 | CP-24 | 12.2, ADR-18, 5.3, 6.1, 4.3 |
| B-22 | Dated dependency register, customer agreement | accept | P1 | CP-49 | 13 columns, OI-5, 1.2 |
| B-23 | Company repository, CI runner, hardened image, network security | accept_modified | P1 | CP-50 | 3, 8, new 8.1, [R8] |
| B-24 | M11 / M12 deferred; JSON-lines weld log | accept_modified | P2 | CP-31 | M11 / M12 rows, D12 / D13 / D18, ADR-07 |
| B-25 | Hardware-stage gate items | accept_modified | P2 | CP-51 | OI-36, 5.2, 5.3, 12.1 SF6, 6.2, A56 |

CP review: CP-24, 25, 26, 27, 28, 29, 31, 32, 33, 34, 35 modified; **CP-30 withdrawn** (ADR-21 rejected for Phase 1).

## 7. Dissent

| Member | Residual dissent |
|---|---|
| CTO | Preferred the batched collision backend inside the spike, so C5 is measured on the production path. Accepted: the backend sits behind an interface and C5 is recorded with the backend used |
| VP Quality & Safety | Wanted the go / no-go touch check committed for R2 now. Accepted: deferred to the HW-1 gate with dated inputs (OI-16 / OI-30 at the Sim-2 exit) |
| Delivery Director | A slip of the TST-002 decision must not become open-ended (capped at 3 working days). Rev 0.6 should stay small; hardware-stage content stays "accepted in principle — decide at gate". Concern about Sim-2 load and Truong's review bandwidth |

## 8. Escalations (decisions needed from people)

| E-id | OI | Question | Options | Board recommendation | Decider |
|---|---|---|---|---|---|
| E-01 | OI-37 | Hold the TST-002 decision on 2026-11-06 with frozen inputs; missing inputs = slip ≤ 3 days, not Pivot? | A yes; B decide on whatever exists, missing = Pivot; C no fixed date | A | Linh Tran, with Truong Hoang |
| E-02 | OI-38 | Amend ADR-17 with a `trial` scope; production = promotion of the same frozen bundle? | A amend now (scope table + CLI refusal before Sim-2, gate code at HW-2); B decide at HW-1; C keep, run A72 / A73 outside M8 / M9 | A, B as fallback; C breaks AD8 | Truong Hoang |
| E-03 | OI-39 | Amend OI-28: named human PLC & safety engineer, commissioning engineer + backup, LSO? | A amend, names by 2026-11-20, LSO before HW-2; B PLC program only; C keep | A — the agent must not write emission code outside PLC sim mode | Linh Tran, with Truong Hoang |
| E-04 | OI-40 | Phase 1 deliverable on 2026-12-04? | A simulation-proven system (Sim-2); B Phase 1 end after HW-2; C compress HW into December | A, aligned with the customer agreement (OI-5) | Tac Huynh and Linh Tran |
| E-05 | OI-41 | Filler wire on J1 / J2, which side? | A autogenous, free ψ; B wire leading, ±15° (Assumption); C unknown: report both, decide on wire case | Answer by 2026-11-04, else C | Truong Hoang (welding engineer, Khang Doan) |
| E-06 | OI-42 | Rebuild IPC / CI image from public `ros:jazzy` before HW-1? | A yes, in parallel, HW-1 entry; B demo image on the IPC; C switch now | A | Truong Hoang |
| E-07 | OI-43 | Runtime per-part IK re-solve as exception to design rule 1? | A no exception; B bounded exception with written bounds; C offline re-plan per part | Decide at HW-1 gate with measured E2; default A | Truong Hoang |
| E-08 | OI-44 | Demo part 564082 or 152056; part-detection sensor in Phase 1? | A 564082 only; B add 152056; C sensor in Phase 1 | A, plus B if STEP by 2026-11-13; C only as staged CP after HW-1 | Tac Huynh |

## 9. Target architecture (summary, Proposed where not decided)

- **Layers unchanged** (L1 Web UI, L2 services, L3 core and data, L4 controllers / devices), plus a CI-checked
  dependency rule: runtime packages depend only on `awm_core`, `awm_interfaces` and the read-only part of
  `awm_motion`.
- **Engineering side (offline).** `awm_api` = one FastAPI process, no rclpy in the Sim stages; long operations run as
  CLI subprocesses with a run folder, one slot per cell, `409` when busy, SIGTERM cancel. OCCT stays in its conda
  subprocess; M1 adds a joint-aware fused weldment shape. M4 is a pure-Python library behind KinematicsBackend and
  SceneBackend (first: `move_group` services); ladder DP with unwrapped joints, configuration labels, ψ inside the WPS
  window, deterministic tie-break, explicit contact check. `move_group` only for OMPL air moves, stored once in D09.
  M6 writes D09 as the exact commanded trajectory; M7 re-verifies the interpolated motion from the frozen bundle in a
  fresh scene.
- **Data.** Job package as Rev 0.5 plus a frozen per-revision release bundle with one active pointer per scope; one
  context record embedded in D07 / D09 / D10 with one compare function and a policy table; D16 split into the hashed
  `cell/config/cell.yaml` and unhashed software defaults; D14 = cell statics, D19 = part / fixture / clamps attached
  to `positioner_flange` plus the scene manifest shared by planner and browser; schemas v2 in mm frozen 2026-11-04.
- **Runtime (Sim-2).** M8 sequencer (scope table, context compare, mixed-mode refusal, JSON-lines weld log), M9 in
  Python sending D09 verbatim, M10 sim driven by the DES-002 weld-event table; typed `awm_interfaces` v1 (weld_axis,
  dwell, expected duration, ω, release SHA, EmissionLost); M8-driven heartbeat; no automatic emission retry;
  positioner as ros2_control component with conformance to D09.
- **Safety boundary unchanged in principle.** The PLC owns emission; SF3–SF5 are process interlocks until REQ-002
  assigns a PLr; a named human authors the PLC program and DES-002 (Proposed, E-03).
- **ADRs.** Confirm (2026-11-06): ADR-01, 02, 05, 13, 15, ADR-07 for the repository. Modified: ADR-08, 12, 17
  (escalated), 18, 19, 20, 22, 23. Rejected: ADR-21.

## 10. Roadmap

| Window | Content |
|---|---|
| 2–6 Nov 2026 (before the TST-002 decision) | Freeze TST-002 Rev 0.3 inputs (B-01…B-05); spike written as the M4 core library in mm; pin `ur-analytic-ik`; 4 Nov: v2 schemas and example files, Sim-1 OpenAPI, minimal D20 exporter, `awm_interfaces` v1 draft, filler-wire answer (E-05); 6 Nov: TST-002 decision (E-01), disposition table, DES-001 Rev 0.6 issued, OI-27 answered |
| To the Sim-1 exit (20 Nov) | T0.1 on kept code (cell.yaml, D08 components, context record, bundle layout); J1 Mode C end to end (CLI 13 Nov, browser 20 Nov); J2 robot-only, 100 % or rejected; D14 / D19 boundary, scripted A18 with default-deny, FK consistency and contact-zone tests; CLI runner; S1-min / S2 / S4-lite; URSim set-up in WP0; customer agreement (13 Nov); named humans and ADR-23 reviewers (20 Nov); ADR-22 + OI-14 (20 Nov); DES-002 weld-event table draft; company remote + CI; DES-001 Rev 0.7 |
| Sim-2 (23 Nov – 4 Dec) | ADR-23 effective; `awm_motion` package and dependency rule; D09 as commanded trajectory; Python M9, M8-minimal, table-driven M10 sim; fault injection and commanded-vs-actual on URSim; independent run by the commissioning engineer; trial scope per E-02; OI-16 / OI-30, DES-002, REQ-002 Rev 0.2, OI-2 / OI-4 |
| HW-1 entry | Human-authored PLC program without emission; ADR-23 on a protected branch; hardened image and 8.1; IK refinement on D14, FK 0.01 mm; UR safety configuration as planning data; drive with STO; Robot Moving threshold measured; 12.1 split; touch-check decision; marker-based cell model; runbooks; DES-001 Rev 0.8 |
| HW-2 entry | LSO, controlled area and SOP, complete enclosure (OI-24), fume interlock; PLC interlock matrix and weld-event conformance at HIL; OI-22; A74 gate code; D12 quality fields and A56; release revocation; OI-36 |
| Phase 2 | MIG; continuous 7-axis motion; A86 and reverse replay; automatic Mode B and A17; seam re-match; re-anchor; M11 / M12 packages; customer remote viewer; STL fallback; Tesseract only if ADR-12 is reversed |

## 11. Next steps

1. Truong Hoang and Linh Tran review this record and the Rev 0.6 draft; decide or reject each B-id (default:
   accepted unless vetoed, per B-14) and the escalations OI-37…OI-44.
2. Answer E-05 (filler wire) by 2026-11-04; freeze the v2 contracts on 2026-11-04.
3. TST-002 Rev 0.3 with frozen inputs (prepared on the Drive master, then copied unchanged into `docs/reference/`).
4. Rebase PLAN.md and prepare DES-003 Rev 0.6 with the impacts listed in the change proposal.
5. Issue DES-001 Rev 0.6 on 2026-11-06 with the TST-002 decision.
6. Wiki updates (wiki session only): board record and escalations in the decisions / open-items registers.
