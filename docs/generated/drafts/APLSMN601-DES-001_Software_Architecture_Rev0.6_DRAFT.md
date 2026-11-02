# APLSMN601 Automated Welding Machine — Software Architecture Description

| Field | Value |
|---|---|
| Doc. No. | APLSMN601-DES-001 |
| Rev | 0.6 (draft) |
| Date | 2026-11-02 |
| Status | Draft for review — prepared by the software agent from the architecture review of 2026-10-30 (CP-24…CP-35) and the consensus of the Architecture Review Board of 2026-10-30 (decisions B-01…B-25, CP-36…CP-51; record `docs/generated/arch_review_board_2026-10-30.md`). The board members were AI reviewer agents (roles, not people); their result is input for the human review by Truong Hoang and Linh Tran. Not approved. Items marked **Proposed** need a decision; the board escalations E-01…E-08 are open items OI-37…OI-44. Planned issue: 2026-11-06, together with the TST-002 decision. Working baseline stays Rev 0.5 until this draft is accepted |
| Prepared by | Truong Hoang |
| Reviewed by | [TBD] |
| Approved by | Linh Tran (Engineering Manager) — pending |
| Standard | Structure follows IEEE 1016 (design viewpoints) |
| Gate | G2 (Design) — WBS 2.4 |

## Revision history

| Rev | Date | Description | By |
|---|---|---|---|
| 0.1 | 2026-10-16 | First draft, built from Function Register APLSMN601-DES-003 Rev 0.1 and the as-built review APLSMN601-DES-004 Rev 0.1 | Truong Hoang |
| 0.2 | 2026-10-19 | ADR-10 added: Phase 1 runs on Ubuntu 22.04 + ROS 2 Humble (existing dev environment); migration to Ubuntu 24.04 + Jazzy planned for Phase 2. Sections 1.4, 4.1, 8 and 13 (OI-9) updated | Truong Hoang |
| 0.3 | 2026-10-22 | Gap analysis against the UR5e Welding Robot Project Tracker [R9] and alignment with Function Register Rev 0.2. ADR-10 changed to Jazzy for Phase 1 (OI-9 closed). New ADR-11 (positioner as Phase 2 slot), ADR-12 (motion planner), ADR-13 (simulated drivers); ADR-04, ADR-06, ADR-09 updated with Tracker alternatives. New functions and data items D17–D19 allocated (Sections 4.2, 5, 11). Weld sequence, speed-scaling rule and PAUSED state (6.1, 6.2). New Section 7.3 user roles and command security. Open items OI-10…OI-16. Appendix A: Tracker cross-reference. Figures 2, 3 and 5 not yet redrawn (see OI-17) | Truong Hoang |
| 0.4 | 2026-10-22 | Decisions of 2026-10-22 (second review): (1) Phase 1 process = laser only, IPG LightWELD 1000 controlled by the PLC; MIG moved to Phase 2 (process abstraction kept). (2) Automatic seam detection removed from scope; seams are picked one by one in the 3D simulator view, once per part number. (3) mm for all project data (ADR-14). (4) ADR-09 decided: Web UI (Section 4.5, API 7.4). (5) ADR-11 extended with the 7-axis synchronisation analysis (Section 6.3). (6) CAD module runs as a separate process (ADR-15). Collision-model data sources (Section 5.4). D20 view model added; A23 deleted; A88 added | Truong Hoang |
| 0.5 | 2026-10-29 | Decisions of 2026-10-27 and document review of 2026-10-29: (1) 1-axis rotary positioner in Phase 1, controlled by ROS 2, Modes B / C (ADR-11, 6.3); (2) cell, fixture, head and positioner geometry from Fusion STEP assemblies with marker components (new ADR-16, A18, D21); (3) placement study A17 and placement source in D19; (4) Z up, seam / beam axis convention, TCP at the nozzle tip (5.3); (5) release scopes simulation / production (new ADR-17); (6) calibration before path planning; (7) tack welds by hand; (8) seam picking by Engineer only; (9) Rev 0.4 inconsistencies corrected (pick_ik, clearance values, wrist-3 limits, D09 / D10 format, launch names, references, frame chain). OI-13 and OI-20 closed, OI-22…OI-28 added; function owners and J2 strategy decided 2026-10-29 (OI-27, OI-28) | Truong Hoang (draft by the software agent) |
| 0.6 (draft) | 2026-11-02 | (A) Architecture review of 2026-10-30 (CP-24…CP-35): accuracy chain (new ADR-18) and error budget (12.2); safety concept, software view (12.1); planning runs (new ADR-19); release integrity (new ADR-20); `awm_api` split (ADR-21, now rejected); Phase 1 positioner interface (new ADR-22, proposed); human review gate (new ADR-23, proposed); Web API rules (7.4); test strategy and observability (12.3, 12.4); quality targets; Sim-1 thin slice; OI-29…OI-35. (B) Architecture Review Board of 2026-10-30 — AI reviewer agents, input for the human review — decisions B-01…B-25 (record `docs/generated/arch_review_board_2026-10-30.md`, CP-36…CP-51; CP-24…CP-29 and CP-31…CP-35 modified, CP-30 withdrawn): TST-002 decision on 2026-11-06 with frozen inputs; joint-aware seam frames; planner state space and acceptance metrics; explicit nozzle contact check without ACM exemption; ψ window per WPS; M4 core behind backends and a runtime dependency rule; one owner per scene object and a shared scene manifest; D16 and D08 split; immutable per-revision release bundle; one context record; CLI job runner (ADR-21 rejected, ADR-19 reduced); ordered Sim-1 slice; staged Web UI and contract freeze; delivery stages Sim-1…HW-2 (new 2.3); D09 as the exact commanded trajectory; typed weld events without automatic retry; minimal Sim-2 runtime; ADR-22 conformance wording; accuracy chain restructured; dated open items; network and host security (new 8.1); M11 / M12 deferred; hardware-stage gate items. Proposed amendments of decided items pending escalation: trial release scope (ADR-17, OI-38), named human PLC / safety roles (OI-28, OI-39). OI-36…OI-44 added; OI-7, OI-8, OI-33, OI-34, OI-35 closed | Truong Hoang (draft by the software agent) |

## 1. Introduction

### 1.1 Purpose

This document defines the software architecture of the APLSMN601 Automated Welding Machine: the software modules, their responsibilities, the data that flows between them, the interfaces to the robot, cell PLC and IPG LightWELD 1000 laser welder, and the runtime and deployment structure. Each function A1–A9 of the Function Register (APLSMN601-DES-003) is allocated to exactly one owning module.

### 1.2 Scope

Phase 1 proof of concept: UR5e on the Pismo frame, **laser welding with an IPG LightWELD 1000** (CW 1000 W, wobble head, wire feeder), **1-axis rotary positioner controlled by ROS 2** (decision 2026-10-27), STEP import, cell and fixture geometry from the mechanical CAD (Fusion), manual seam selection in the 3D simulator view, offline path generation, simulation, calibration, PLC-controlled laser, Web UI. Phase 2 items (MIG process, STL fallback A16, multi-pass A46, continuous 7-axis motion with the positioner, pause / resume A86, seam tracking A94) are shown only as extension points. Automatic seam detection is not part of the product scope (decision 2026-10-22). Phase 1 deliverable on 2026-12-04 = Sim-2 (2.3) unless OI-40 (escalation E-04) decides otherwise; HW-1 / HW-2 dates [TBD] in PM-004. Phase 1 demonstration scope per the customer agreement (OI-5, due 2026-11-13).

### 1.3 References

| ID | Document |
|---|---|
| [R1] | APLSMN601-PM-001 Project Charter Rev 0.2 |
| [R2] | APLSMN601-PM-003 Work Breakdown Structure Rev 0.1 |
| [R3] | APLSMN601-DES-003 Function Register Rev 0.5 (source of truth for functions) |
| [R4] | APLSMN601-DES-004 Function Register As-Built (Tac Huynh ROS2 demo) Rev 0.1, findings F01–F15 |
| [R5] | APLSMN601-DES-002 Interface Control Document (planned) — signal-level detail of Section 7 |
| [R6] | APLSMN601-REQ-001 Software Requirements Specification (planned) |
| [R7] | APLSMN601-REQ-002 Risk Assessment ISO 12100 (planned) |
| [R8] | ISO 10218-1/-2 (edition 2025, incl. cybersecurity), ISO 12100, ANSI Z136.1, ISO/IEC/IEEE 12207, IEC 62443-3-3, IEC 60825-1, IEC 60825-4 |
| [R9] | UR5e_Welding_Robot_Project_Tracker (Google Sheet, based on the project roadmap document of 15 Oct 2026): Pipeline, Architecture, Module Details 5.1–5.9, Tasks W0–W9, Assumptions A1–A6, Open Questions Q1–Q7, Risks R1–R9 |
| [R10] | APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.2 (results and frozen inputs in Rev 0.3, B-01) |
| [R11] | IPG LightWELD 1000 W specification (distributor data sheet) and LightWELD XR user manual (Section 5.6 customer interface connections, 12-pin connector) — checked 2026-10-22; IPG automation interface documentation still to be obtained (OI-2) |
| [R12] | Cell CAD `APLSMN601_Automated Weldment System.step` (Fusion export, 2026-10-27): enclosure lower frame, Pismo plate, UR5e, LightWELD head and mount, 1-axis positioner, jig and holders, golden part `564082-Assembly-3 parts` |
| [R13] | Change proposal DES-001 (`docs/generated/DES-001_change_proposal.md` in the software repository): Rev 0.4 → Rev 0.5 CP-9…CP-23; Rev 0.5 → Rev 0.6 CP-24…CP-35 and, from the Architecture Review Board, CP-36…CP-51 |
| [R14] | Universal Robots `ur_calibration` (part of `ur_robot_driver`): extraction of the factory-calibrated kinematics of one robot into a `kinematics_config` file for `ur_description` |
| [R15] | Architecture Review Board record of 2026-10-30 (`docs/generated/arch_review_board_2026-10-30.md`): AI reviewer agents, decisions B-01…B-25, escalations E-01…E-08 — input for the human review |

### 1.4 Terms

| Term | Meaning |
|---|---|
| IPC | Industrial PC running Ubuntu 24.04 + ROS 2 Jazzy (ADR-10) (planning and cell software) |
| Cell PLC | Beckhoff TwinCAT 3 controller (cycle, interlocks, LightWELD I/O, safety) |
| LightWELD | IPG LightWELD 1000 fiber-laser welder: base unit + weld head (1.3 kg) + 10 m fiber + wire feeder; parameters stored as presets |
| Wobble | Oscillation of the laser beam made by the weld head (amplitude, frequency) — replaces robot weaving |
| Job | One part number + revision with its seams, parameters, paths and programs |
| Workbench | Engineering application used to create and release jobs (offline) |
| Runtime | Software that runs released jobs on the cell (online) |
| D01…D21 | Data items of the Function Register Rev 0.5 |
| ψ (tool yaw) | Rotation of the weld head about its beam axis — redundant DOF used for reach, collision and singularity avoidance; redundant only inside the ψ window of the WPS (filler wire side, tip type; B-05) |
| ACM | Allowed Collision Matrix (MoveIt) |
| CTWD | Contact-tip-to-work distance (stick-out) |
| Positioner | 1-axis rotary positioner (7th, external axis) carrying fixture and part — Phase 1, controlled by ROS 2 (ADR-11) |
| Mode B / Mode C | Positioner motion modes of Phase 1 (6.3): B = rotate, stop, weld; C = robot at rest, positioner turns during emission (seam coaxial with the positioner axis) |
| Marker component | Small named component in a CAD assembly whose origin and axes define a frame (`REF_*`, ADR-16) |
| Release scope | `simulation` or `production` (ADR-17); `trial` Proposed (B-16, OI-38) |
| ADS | Beckhoff Automation Device Specification protocol (TCP 48898) |
| Error budget | List of all position error sources between the planned TCP pose and the real seam, with their values and combined result, compared with the allowed seam deviation (12.2) |
| Safety function (SF) | Function whose failure can increase a risk (ISO 13849-1); required performance level PLr from the risk assessment REQ-002 |
| Process interlock | Interlock in the standard PLC that protects the part or the process; not credited as a safety function until REQ-002 assigns a PLr and a safety-rated realisation (12.1) |
| Planning run | One execution of the planning CLI for a job, identified by `run_id`, with its run folder, inputs, software version and result (ADR-19) |
| Context record | One record built by `awm_core` with the cell, calibration, robot and software identity of a plan or program; embedded in D07, D09, D10 and compared at run time (5.2, B-10) |
| Release bundle | Frozen copy of all inputs of one released program revision in `programs/<rev>/bundle/` (ADR-20) |
| Delivery stage | Sim-1, Sim-2, HW-1, HW-2 or Phase 2, with entry and exit criteria (2.3) |

## 2. Architecture drivers

### 2.1 Requirements that shape the architecture

| # | Driver | Source | Architectural consequence |
|---|---|---|---|
| AD1 | Weld paths from CAD without teaching (O1) | Charter | Exact B-Rep geometry (OCCT), not mesh; offline planning chain A1→A4→A6 |
| AD2 | Many products in one cell (O5) | Charter | No part-specific code. All part data lives in a job package; one generic pipeline |
| AD3 | Laser welding with IPG LightWELD 1000 in Phase 1; MIG later (O2, O4; decision 2026-10-22) | Charter, decision | Process abstraction kept: one weld-event model; Phase 1 has only the LightWELD driver in the PLC. Weaving is done by head wobble, not by the robot |
| AD4 | Robot ↔ LightWELD ↔ PLC communication (O3) | Charter, decision | PLC is the single owner of LightWELD trigger / enable / preset signals and of the laser interlocks |
| AD5 | Accuracy ±0.1–0.2 mm (laser; wobble may widen the allowed band — OI-16) | Charter risk 02 | Explicit frame chain (part → fixture → base → TCP) with calibration data D08; robot-specific calibrated kinematics mandatory for every motion on hardware (ADR-18); no runtime per-part correction in Phase 1; error budget by direction and mode (12.2) |
| AD6 | Class 4 laser hazard (1070 nm, invisible) and fumes | Charter risk 06, REQ-002 | Safety is hardwired and independent of the IPC software |
| AD7 | 2-month Phase 1 timeline (early Dec 2026) | Design Review deck | Reuse the ROS2 / MoveIt stack of the Tac demo where it is sound |
| AD8 | What is verified is what runs | DES-004 F01, F07 | Programs are planned and verified offline, stored, released and replayed — no weld-path planning at runtime |
| AD9 | 360° pipe seams (customer parts: elbows, pipe nipple, flange — 4…\3. MECHANICAL\StepFiles) | Customer STEP files, [R9] 5.5 | Tool-yaw DOF must stay free in planning (A44); 1-axis positioner in Phase 1 for seams coaxial with its axis (ADR-11, decision 2026-10-27) |
| AD10 | Hardware not yet available (LightWELD on loan, PLC, positioner) | [R9] design principles, R5 | Every hardware interface has a simulated driver with the same interface (ADR-13) |
| AD11 | Seams chosen by people, not by software | Decision 2026-10-22 | 3D view must expose every B-Rep edge with a stable ID; picking and seam editing are core UI functions (Section 4.5) |
| AD12 | Cell, fixture, head and positioner designed in Fusion by the mechanical team | Decision 2026-10-27 | Nominal geometry and frames are imported from the CAD assembly, never typed by hand (ADR-16); measured calibration replaces nominal values on hardware |

### 2.2 Design principles

1. **Offline / online split.** Engineering (A1–A4, A6, A7) produces a released, revision-locked program. The runtime (A8) only executes released programs.
2. **Data-driven, not code-driven.** Seams, process parameters, poses, I/O maps and cell geometry are data files with versioned JSON schemas. Changing a part never requires a code change (fixes F03).
3. **One owner per function and per data item.** Every function ID maps to one module; every data item has one producer.
4. **Hardware safety first.** E-stop, guard doors and the laser interlock chain work without the IPC. Software can stop the cell but never overrides safety.
5. **Fail at planning, not at welding.** A seam that is not 100 % reachable and collision-free is rejected in the Workbench (fixes F01).
6. **No absolute paths, one config source.** All paths come from the job repository root and ROS2 package share paths; each parameter is defined once (fixes F06, F08).
7. **Simulate first.** Code runs against simulated drivers from day one; switching to hardware is a launch parameter, not a code change (ADR-13). The launch parameter selects drivers; M8 derives the mode from the connected drivers and refuses a mix (a declared `hil` mode only while the PLC reports emission disabled by hardware; B-19).
8. **mm everywhere in project data** (decision 2026-10-22, ADR-14). All project-owned data, JSON schemas, `awm_interfaces` messages, UI and reports use mm, degrees and seconds. Third-party ROS packages (ur_description, MoveIt2, TF, ur_robot_driver) are fixed to metres and radians by REP 103; the single mm ↔ m adapter in M0 is the only place that converts.
9. **Thin slice first.** Build one end-to-end slice (one part, one seam, one WPS) through the whole chain before widening features. **Sim-1 slice (CP-34 as modified by B-12, Proposed; order and acceptance in PLAN.md):** golden part 564082; J1 Mode C first; J2 robot-only, accepted as a 100 % path or a rejection with failing points; screens S1 (minimal), S2 and S4 (lite) (4.5).
10. **Independent human review of safety-relevant work** (Proposed, ADR-23, effective 2026-11-23). Code and data that can move the robot or positioner, request emission or define what counts as verified (M8, M9, M10, `awm_positioner`, `awm_interfaces`, the cell part of D16, the A18 / D14 generator including the contact model, the M4 acceptance rules and the M7 checks) are merged only after review by a named engineer. During Sim-1 Truong Hoang signs off the diff of the verification-defining files at each acceptance run. The Cell PLC program needs a named human author, not only a reviewer (B-18, OI-39).

### 2.3 Delivery stages (B-14, Proposed)

One stage scheme for all documents. R1–R5 of the charter are kept as reference (Section 11). DES-001 changes only at stage gates: Rev 0.6 issued 2026-11-06 with the TST-002 decision, Rev 0.7 at the Sim-1 exit, Rev 0.8 at the HW-1 gate. Between gates, changes are CPs with a status.

| Stage | Content | Entry | Exit | Acceptance run |
|---|---|---|---|---|
| Sim-1 | Offline chain on mock hardware plus browser (S1 minimal, S2, S4 lite); J1 Mode C, J2 robot-only (principle 9) | TST-002 decision (OI-29); v2 data contracts frozen 2026-11-04 | Golden-job planner regression and FK consistency test (planner vs browser) green; simulation release written | Scripted run by Truong Hoang: import → pick → plan → program → verify → playback, logged in PROGRESS.md |
| Sim-2 | Simulation release replayed by M8 / M9 / M10 sim on URSim plus the mock positioner | Sim-1 exit; `awm_interfaces` v1 frozen; ADR-23 effective 2026-11-23 | Fault-injection subset and commanded-vs-actual on URSim pass; independent run by the commissioning engineer (OI-39) | Sim-2 acceptance script (B-19) |
| HW-1 | Calibration (A51–A55) and dry run (A72), emission blocked by hardware | DES-002; PLC program without emission by a human author; ADR-23 active; REQ-002 Rev 0.2; IPC and drive installed; UR safety configuration designed, with its checksum; drive STO wired; hardened image (OI-42); OI-16 / OI-30 values; IK refinement on D14; independent Sim-2 run passed | Calibration and dry-run records; touch-check decision (12.2) | Runbook-based dry run by the commissioning engineer |
| HW-2 | Trial weld (A73) | LightWELD on site (OI-21); Laser Safety Officer named; Class 4 controlled area and written SOP; enclosure complete incl. OI-24; fume extraction interlocked; PLC interlock matrix and weld-event conformance PASS; OI-22 decided | Trial-weld records; production gate (A74) | Trial-weld campaign (release scope per OI-38) |
| Phase 2 | MIG, continuous 7-axis motion, pause / resume (A86), automatic Mode B and A17, seam re-match, M11 / M12 packages, STL fallback | — | — | — |

## 3. Context viewpoint

![Figure 1 – System context](APLSMN601-DES-001_figures/fig1_context.png)

| External actor / system | Interaction with the software |
|---|---|
| Customer | Supplies STEP files (D01) and drawings with weld symbols |
| Process / robot engineer | Creates jobs in the Workbench, calibrates the cell, releases programs |
| Welding engineer | Maintains the WPS library (D05) |
| Operator | Loads parts, selects job, starts cycle, inspects welds (A81, A91) |
| UR5e controller (e-Series, PolyScope 5.x) | Executes joint trajectories through ur_robot_driver / External Control URCap; safety configuration |
| Cell PLC (Beckhoff TwinCAT 3) | Cycle permissives, LightWELD control, alarms, safety I/O |
| IPG LightWELD 1000 (+ wire feeder) | Trigger / emission enable, preset select, ready / fault, external interlock loops — via PLC (exact signal set per IPG automation documentation, OI-2) |
| Rotary positioner (motor drive) | Joint trajectory from ROS 2 (`awm_positioner`); drive ready / fault hard-wired to the PLC; STO from the safety chain |
| Mechanical design (Fusion) | Supplies cell, fixture, head and positioner CAD as STEP assemblies with marker components (D21) |
| Customer / remote viewer | Phase 2; Phase 1 reports via the shared drive (B-13) |
| Git server | Source code (Rule 4 of the project README) on an Apillis-controlled private remote with protected branches and CODEOWNERS (B-23). Until the remote is confirmed private, customer CAD and job data are git-ignored and nothing is pushed |
| Shared drive | Documents, released program archive copies, reports |

## 4. Composition viewpoint (logical architecture)

### 4.1 Layers

The software is organised in four layers. Higher layers depend only on lower layers.

| Layer | Content | Runs on |
|---|---|---|
| L1 Presentation | Workbench UI (engineering), Operator HMI | IPC (or engineering laptop for Workbench) |
| L2 Application | Engineering services M1–M7, Runtime services M8–M12 | IPC, ROS 2 Jazzy |
| L3 Core & data | Data model, JSON schemas, frame math, job repository, cell model | IPC (pure Python / C++ libraries) |
| L4 Control & field | UR controller, Cell PLC program, LightWELD, safety | UR controller, Beckhoff PLC, devices |

**Dependency rule (Proposed, B-06).** The runtime packages `awm_cell`, `awm_executor`, `awm_plc_bridge` and `awm_positioner` depend only on `awm_core`, `awm_interfaces` and the read-only part of `awm_motion` (FK, D09 → JointTrajectory conversion, override time scaling, start-state check). They never depend on `awm_path`, `awm_verify` or MoveIt clients. A CI test checks `package.xml` and the Python imports.

![Figure 2 – Module architecture](APLSMN601-DES-001_figures/fig2_modules.png)

### 4.2 Module catalogue

| Module | ROS2 package / project | Responsibility | Functions owned | Phase |
|---|---|---|---|---|
| M0 Core | `awm_core` | Data classes, JSON schemas (D03–D13, D16–D19), frame transforms, job repository API, mm ↔ m adapter for third-party ROS packages (ADR-14); owns the D16 cell-configuration schema (B-08), the context record and its compare function (B-10), the record and weld-log writers (B-24) | — (shared) | 1 |
| M1 CAD Import | `awm_cad` | Load STEP AP203/214/242 with OCCT, split solids, normalise units, check and heal, tessellate (glTF display + collision mesh), create part record with file hash, export the 3D view model D20 (glb per face / edge + topology), place part in the virtual cell (D19, source `study` or `fixture_cad`); import the cell / fixture CAD assemblies (D21) into the cell model D14 and the nominal calibration (A18); STL fallback with RANSAC cylinder fit; joint detection between solids (general fuse; each face keeps its source-solid id, B-02). Runs as a separate process (ADR-15) | A11, A12, A13 (render), A14, A15, A17 (with M4), A18 · A16 (Phase 2) | 1 / 2 |
| M2 Seam Definition | `awm_seam` | Resolve picked edge IDs to exact B-Rep curves (a picked edge on either solid is resolved to the joint edge; n1 / n2 come from the exterior faces of both solids, B-02), chain tangent-continuous edges into one seam, seam attributes (joint type, closed / open, side, start point, direction, split, sequence, leg size; tack welds are made by hand, A81), seam list export with face normals; analytical T / L pipe seam generator as test tool | A21, A22, A24, A25 (test) | 1 |
| M3 Process Parameters | `awm_process` | WPS library maintenance with versioning (released WPS read-only), laser mode per seam (CW / HPP pulse), LightWELD preset and parameter assignment | A31, A32, A33, A34 | 1 |
| M4 Path Planner | `awm_path` | Core library behind KinematicsBackend / SceneBackend (B-06): seam discretisation and seam frame, head orientation and focus offset (wobble is a head parameter, no robot weave), approach / retract and air moves, IK with tool-yaw and IK-branch graph search (ADR-12), collision check, time parameterisation at WPS travel speed in the part frame, wrist-3 soft limit / cable-wrap check, positioner mode and angles (Modes C / B); multi-pass in Phase 2 | A41–A45, A47–A49 · A46 (Phase 2) | 1 / 2 |
| M5 Calibration | `awm_calib` | TCP calibration (4–6 orientations, least squares), fixture frame (3-point), part location (fixed-jig check, touch sensing later), error budget, positioner axis, calibration records; import of the robot-specific kinematics (`ur_calibration`, ADR-18); part location in Phase 1 as a go / no-go check with a typed LOCATE_PART result (B-21); TCP verification A56 (HW-2, B-25) | A51–A55 | 1 |
| M6 Program Generator | `awm_program` | Transform path to base frame, build executable program (joint trajectory segments + weld events), store with revision; writes the exact commanded trajectory (positions, velocities, accelerations and times per controller, B-15) | A61, A62, A63, A64 | 1 |
| M7 Verification & Release | `awm_verify` | URSim replay = M8 / M9 runtime chain in Sim-2 (B-19); playback in the Web simulation view, reach / collision / cycle-time report, dry-run and trial-weld records, release workflow (scopes `simulation` / `production`, ADR-17; `trial` Proposed, OI-38) and lock into a frozen per-revision bundle (ADR-20); rebuilds the scene from the frozen bundle and self-tests it with the C1 pose set before verification; verifies the controller-interpolated motion at the controller rate (B-06, B-15) | A71, A72, A73, A74 | 1 |
| M8 Cell Sequencer | `awm_cell` | Cycle state machine, job selection, start / stop / reset, alarm handling and recovery, speed-scaling supervision; pause / resume with overlap in Phase 2; mode derived from the connected drivers (B-19); Phase 1 weld log as append-only JSON lines with a start and an end record per seam (B-24) | A81, A82 (supervisor), A84 · A86 (Phase 2) | 1 / 2 |
| M9 Motion Executor | `awm_executor` | Replay released trajectory segments on the UR5e through ur_robot_driver and on the positioner through its joint trajectory controller, start-state check, speed override for air moves only, speed-scaling monitor, feedback; sends D09 points verbatim, override time scaling only on air / index segments (B-15); Python in Phase 1 (ADR-08) | A83 (motion) | 1 |
| M10 PLC Bridge | `awm_plc_bridge` | ADS link IPC ↔ PLC: commands, weld parameters, status, alarms, heartbeat; `sim` mode emulates PLC + LightWELD with the same interface (ADR-13); LightWELD only — the positioner is not behind the PLC (ADR-11); sim mode table-driven from the DES-002 weld-event behaviour table (B-18) | A82 (IPC side), A63 (runtime events) | 1 |
| M11 Weld Logger | `awm_logger` | **Phase 2 package** (B-24); Phase 1 realised by the M8 JSON-lines weld log and `awm_core` record types. Phase 2 content: Structured weld log per seam (part, program rev, set vs actual parameters, alarms, times, operator), rosbag2 run recording, part report export | A85, A87 | 1 |
| M12 Inspection & Feedback | `awm_inspect` | **Phase 2 package** (B-24); Phase 1 realised by the record types inspection / ncr / macro_etch in `awm_core`. Content: Inspection record (visual and macro-etch), NCR link, correction request back to A2–A4 (new job revision) | A91, A92, A93, A95 · A94 (Phase 2) | 1 / 2 |
| UI Web front-end | `awm_web` (React + TypeScript, react-three-fiber, urdf-loader) | One browser app with role-based screens: job library, 3D seam picker, simulation, WPS / preset library, production HMI, calibration, reports, users (Section 4.5) | A13, A17, A21, A22 (UI), A71 (display), A74, A81, A84 (display), A87, A88 (UI) | 1 |
| UI backend | `awm_api`: FastAPI, one process, no rclpy in the Sim stages; long operations as CLI subprocesses (ADR-19); ROS bridge decided at HW-1 (ADR-21 rejected) | REST + WebSocket API (7.4), authentication and roles (A88), command checks, job repository access, one planning-run slot per cell (ADR-19), calls to M1–M12 | A88 | 1 |
| Cell model | `awm_description` | URDF / xacro: UR5e (`ur_description`) + LightWELD head and mount (incl. fiber / wire-feed conduit section near the head for collision) + Pismo frame + positioner joint with its permanent holders, SRDF, MoveIt config (fixture and clamps are per job in D19, B-07); xacro macros and loader; cell instance data generated by A18; static geometry, head and positioner links generated from the Fusion CAD by A18 (ADR-16) | D14 | 1 |
| Bring-up | `awm_bringup` | Launch files: `sim_mock` (Sim-1, mock hardware), `sim_ursim` (Sim-2, URSim + mock positioner), `real` (UR5e + positioner drive), `workbench`, `web` (awm_api + static front-end); RViz only for developer debugging | — | 1 |
| Interfaces | `awm_interfaces` | ROS2 msg / srv / action definitions (Section 7.1) — the contract between modules, reviewed by the team before coding; contracts frozen per stage (v2 data contracts on 2026-11-04; `awm_interfaces` v1 at the Sim-1 exit); after a freeze only MINOR additions, otherwise via CP (B-13, B-17) | — | 1 |
| Motion library | `awm_motion` (module of `awm_path` in Sim-1; package before Sim-2) | Scene assembly from files (D14 + D19 scene manifest), FK, D09 → JointTrajectory conversion, override time scaling, start-state check, dense check (B-06) | — (used by M4, M6, M7, M9) | 1 |
| Positioner driver | `awm_positioner` | ros2_control hardware interface for the rotary table (7th joint) in the same controller manager as the UR driver; `mock_components` in simulation; drive on EtherCAT proposed (OI-14) | A83 (positioner motion), A55 (runtime) | 1 |
| Cell PLC program | TwinCAT project `APLSMN601_CellPLC` (separate Git repo) | Cycle permissives, robot handshake, LightWELD driver (trigger, preset, interlocks, wire feeder), positioner drive ready / fault monitoring (emission off on fault), safety supervision, alarms. Owner / author: named PLC & safety engineer (OI-4; Proposed, OI-39); the software agent drafts requirements only | A81, A82, A84 (PLC side), A83 (process) | 1 |

### 4.3 Engineering chain (offline)

M1 → M2 → M3 → M4 → M6 → M7 runs in the Workbench. Each step writes its output into the job package and records the hash of its inputs, so a later step can detect stale inputs (for example, a seam list that was edited after the path was planned).

The M4 core is a pure-Python library (no rclpy) behind two interfaces (B-06): KinematicsBackend (analytic UR IK seeds, refinement, FK) and SceneBackend (batch validity, clearance, closest pair). Phase 1 SceneBackend = `move_group` services with the cell model D14 and the collision scene D19; a batched in-process backend is added only if C5 > 30 s per seam (measured). `move_group` plans OMPL air moves only; the scene is reset and hash-checked before each run. The Workbench starts its own planning-only ROS2 graph (no robot hardware); this reuses the MoveIt configuration of the Tac demo. IK is enumerated with an analytic UR solver (all 8 branches, ADR-12); pick_ik returns one solution and is not used for the sweep. Planning uses the active calibration D08; a calibration change makes D07 stale (plan after calibration).

M4 process-motion algorithm (ADR-12, validated by TST-002):

1. Discretise the seam by arc length (1–3 mm); seam frame X = tangent, Z = bisector of the face normals pointing out of the material (up, towards the head; for a butt joint between tubes n1 = n2 and Z is the surface normal), Y = Z × X; the beam axis is −Z (A41). Pitch Δs ≤ min(3 mm, Δq_max / rotation rate of the seam frame), about 1.16 mm at r = 13.3 mm (B-03). For joints between separate solids, n1 and n2 are taken from the exterior faces of the two solids (joint-aware topology, M1, B-02).
2. Apply work / travel angle and focus offset (wobble is set in the LightWELD preset, not by the robot) and the 360° closing overlap (A42). The ψ window comes from D06 (B-05).
3. Per point, sample tool yaw ψ inside the WPS ψ window (10° steps; 5° in windows narrower than 60°, B-05) × the 8 UR5e IK branches; analytic IK seeds are refined against the active kinematics (D14) to ≤ 0.01 mm / 0.01°, within the seed's configuration label (B-21, required from HW-1). Drop states in collision, beyond joint limits, whose contact check fails (5.4, B-04), or near a singularity: |sin q5| ≥ sin ε_w, |sin q3| ≥ sin ε_e, wrist-centre distance from the base Z axis ≥ d_s (D16; defaults 8.6°, 5°, 50 mm — Assumption, B-03) (A44, A45). With the positioner (A49): seams whose axis is within 0.05 mm / 0.1° of the positioner axis (D16, Assumption; residual eccentricity → E10, B-03) use Mode C (one robot pose, the positioner turns; the head is checked against the rotating part and holders over the full turn plus the overlap, with the contact check); other seams: robot-only first; Mode B with split points and positioner angles entered by the Engineer (R1); automatic Mode B search in R3 (B-12).
4. Layered graph (layer = point, node = valid state (ψ, configuration label computed from the joint values, unwrapped joint vector), edges only between states with the same configuration label and max |Δq| ≤ 5° on unwrapped joint values; 2π offsets enumerated within the joint limits at the first layer; in closed seams the overlap connects back to the first-layer state; ties broken by cost, then ψ index, then label order (B-03)), edge cost = joint change + clearance and singularity penalties + wrist-3 travel; shortest path gives a continuous chain without configuration flips.
5. Smooth, time-parameterise at the WPS travel speed in the part frame (A47), check wrist-3 on unwrapped values: ±270° absolute soft limit and ±180° travel within one seam (A48). Mode C: positioner speed = travel speed / seam radius; ramps planned by A47 inside the dwell or the closing overlap (B-20).
6. Approach / retract 30–50 mm along the beam axis; OMPL air moves between seams from the chosen joint states and positioner index moves with the robot parked; every segment checked densely: the step is chosen so that no robot or head point moves more than `max_check_displacement_mm` (D16) between checked states; positioner steps Δθ ≤ 2·(c − margin) / r_max of the rotating set; M4 and M7 compute the bound independently (B-03) (A43).

A seam that fails any step is rejected with the failing points listed (shown red in the simulation view); the engineer then changes direction, start point, head angles or part placement (A15, A17, A22, A42).

**Planning runs (ADR-19, Proposed, B-11).** A plan request starts the planning CLI as a subprocess with a `run_id` and a run folder `jobs/<id>/runs/<run_id>/` (command line, input hashes, context record, seed where applicable, JSON-lines progress, start / end, exit status OK / FAIL / CANCELLED). One run per cell, held in memory by `awm_api`; a second request returns `409` with the active `run_id`; job edits are refused during a run; no queue and no lock files; cancel = SIGTERM, then SIGKILL (atomic writes: a cancelled run leaves no D07). Before the run the runner checks that `move_group` is up and that the scene hash matches. Weld, approach and retract segments are identical for identical inputs within one image digest; air moves are planned once, stored and re-verified. A plan made from a dirty source tree can be released only with simulation scope. If the measured scene build per run exceeds 10 s (Assumption), the Sim-1 exit decides on a long-lived ROS 2 action worker behind the same HTTP contract.

### 4.4 Runtime chain (online)

M8 Cell Sequencer is the single coordinator. It loads a released program through the active pointer of its scope (D10; scope `simulation` only in simulation mode, ADR-17), checks the frozen release manifest, the D09 SHA-256 and the context record against the active context (policy table; B-09, B-10), then runs the seam loop (Section 6.2) by calling M9 for motion (robot and positioner) and M10 for weld events. M11 records everything. The Operator HMI talks only to M8.

### 4.5 User interface design (ADR-09: Web)

**Structure.** Browser (`awm_web`) ⇄ HTTPS REST + WebSocket ⇄ `awm_api` (FastAPI) ⇄ CLI subprocesses of M1–M7 in the Sim stages; ROS bridge from HW-1 (B-11). The browser never calls ROS directly. Browser and planner build the scene from the same D19 scene manifest and the D14 URDF (B-07). The physical Start button is wired to the PLC.

**3D view model (D20).** M1 exports for every part revision:

- `view.glb` — one mesh node per B-Rep face (`face_id`) and one polyline node per B-Rep edge (`edge_id`), in mm, edges tessellated with chord ≤ 0.05 mm;
- `topology.json` — edge → adjacent faces, edge length, closed flag, tangent continuity to neighbouring edges, geometric fingerprint (used to re-match seams when the STEP revision changes); joint edges with their solid pair (B-02).

The browser picks only IDs; `awm_api` sends the IDs to M2, which resolves them to the exact B-Rep curve. Display geometry is never used as a weld path.

**Screens and roles**

| # | Screen | Main actions | Roles | Stage (B-13) |
|---|---|---|---|---|
| S1 | Job library | Create job from STEP, open, copy, revision list, status (draft / simulated / released) | Engineer, Admin; Operator read | Sim-1 (minimal: list, create) |
| S2 | Seam picker (3D simulator) | Click edge → auto-chain tangent edges → seam; select / deselect, reverse direction, start point, split, order, joint type; seam frame and beam axis drawn at the start point and every 90° (B-02); cell, fixture, positioner and robot shown in the same scene | Engineer | Sim-1 |
| S3 | Process | Assign WPS / LightWELD preset per seam, focus offset, travel speed, wobble values (display of preset) | Engineer | CLI / API in Phase 1; screen Phase 2 |
| S4 | Simulation | Plan request, trajectory playback with scrubber, points coloured by status (OK / collision / out of reach / near singularity / joint limit / contact) with the colliding link / object, positioner mode (B / C) and angles, cycle time. R3: robot configuration (shoulder / elbow / wrist −1 / 0 / +1) and tool-yaw override per seam, placement study (A17) — edited in JSON until then | Engineer | Sim-1 (lite) |
| S5 | Release | Review report D15, dry-run and trial-weld records, approve → lock | Engineer request, Admin approve | CLI / API in Phase 1; screen Phase 2 |
| S6 | Production HMI | Select released job, Start request, Stop, Reset, seam progress, live LightWELD status, alarms EN / VN | Operator | HW-1 |
| S7 | Calibration | Guided TCP, fixture and positioner-axis calibration, error budget, history | Admin | CLI / API in Phase 1; screen Phase 2 |
| S8 | Reports & logs | Weld log, part report export (D18) | All (read) | CLI / API in Phase 1; screen Phase 2 |
| S9 | Users | Users, roles, permissions | Admin | CLI / API in Phase 1; screen Phase 2 |
| S10 | Cell model | Import the cell / fixture CAD (D21), marker and CAD-map check report | Admin | CLI / API in Phase 1; screen Phase 2 |

**Build steps (mapped to the Function Register)**

| Step | Content | Functions | Release |
|---|---|---|---|
| UI-0 | API contract (7.4), D20 format, seam schema in mm; `awm_api` skeleton with login | A13, A24, A88 | Sprint 0 |
| UI-1 | 3D viewer: part (D20) + cell + robot URDF | A13, A15 | R1 |
| UI-2 | Seam picker S2 | A21, A22 | R1 |
| UI-3 | Simulation S4 with failing-point colours, configuration / tool-yaw override, positioner | A44, A45, A49, A71 | R1 |
| UI-4 | Process S3 + Release S5 | A33, A34, A74 | R2 |
| UI-5 | Production HMI S6 | A81, A84 | R2 |
| UI-6 | Calibration S7, Reports S8 | A51–A54, A87 | R2 / R4 |
| UI-7 | Users S9, permission tests | A88 | R2 |

## 5. Information viewpoint (data)

### 5.1 Data items

Data items D01–D21 are those of the Function Register Rev 0.5. All lengths in mm (ADR-14). The as-built merge of D04 and D07 (DES-004) is reversed: seams and paths are separate files.

| ID | Data item | Producer | Consumers | Format / schema | Storage |
|---|---|---|---|---|---|
| D01 | STEP file | Customer | M1 | .step AP203/214/242 | job/`model.step` (copy, read-only) |
| D02 | B-Rep model | M1 | M1, M2, M4 (collision mesh) | OCCT TopoDS_Shape in memory; .brep cache | job/`cache/` |
| D03 | Part record | M1 | M6, M7, M11 | `part.schema.json` | job/`part.json` |
| D04 | Seam list | M2 | M3, M4 | `seams.schema.json` (curve, face normals n1 / n2 with source-solid ids (B-02), closed / open, attributes, fingerprint) | job/`seams.json` |
| D05 | WPS library | Welding engineer | M3, M4, M7 | `wps.schema.json` (one file per WPS; references the LightWELD preset number and its parameters: power, mode, wobble, wire feed, gas; filler_wire, wire_side, psi_window_deg, tip_type — Assumption until the welding engineer gives values, B-05) | cell/`wps/` |
| D06 | Seam process parameters | M3 | M4, M6 | `process.schema.json` | job/`process.json` |
| D07 | Weld path (part frame): tool poses in mm, timed joint path incl. positioner angle and mode, per-point status, context record (B-10), `run_id` of the planning run (ADR-19) | M4 | M6, M7 | `path.schema.json` | job/`path.json` |
| D08 | Calibration data (TCP, fixture frame, part offset, positioner axis; `NOMINAL` from the cell CAD until measured) | M5 (nominal: M1 / A18) | M4, M6, M8, M9 | `calibration.schema.json`: components kinematics, positioner_axis, fixture, tcp (one per tip type), each with its own SHA-256 (B-08, B-25) | cell/`calibration/<date>_<id>.json` + `active.json` link |
| D09 | Robot program | M6 | M7, M9 | `program.schema.json`: JSON, one segment per controller (robot or positioner) with time_from_start, positions, velocities and accelerations exactly as commanded, hold values of the other axes, weld events and the context record; self-contained (B-15, B-09); URScript export = fallback | job/`programs/<rev>/program.json` |
| D10 | Released program | M7 | M8, M9 | `release.schema.json` (program rev, manifest = SHA-256 of every file of the frozen bundle (ADR-20), context record, scope `simulation` / `production` (`trial` Proposed, OI-38), approver, date) | job/`programs/<rev>/release.json` + job/`active_<scope>.json` |
| D11 | I/O & handshake signal list | DES-002 (design) | M6, M10, PLC | `io_map.yaml` generated from the ICD sheet | cell/`io_map.yaml` |
| D12 | Weld log | M8 (Phase 1), M11 (Phase 2) | M12 | JSON lines, append-only, start and end record per seam (Phase 1, B-24); SQLite (Phase 2); quality-record fields at HW-2; each record carries the release hash and the context record | `logs/weld_log.jsonl` (Phase 1), `logs/weld_log.sqlite` (Phase 2) |
| D13 | Inspection record | M12 | M12 (A93) | `inspection.schema.json`; record types inspection / ncr / macro_etch (B-24) | job/`inspections/` |
| D14 | Cell model | M1 (A18) from D21 + `ur_description` | M4, M7 | URDF / xacro / SRDF — cell statics only: UR5e, head / mount / conduit, frame, positioner kinematics and permanent holders (B-07) | `awm_description` package |
| D15 | Simulation / verification report | M7 | M7 (A74) | JSON + PDF / screenshots; per segment minimum distance, closest pair and margin (B-06); validity statement: nominal geometry, no dynamics, no contact force, no heat, safety slow-downs tested only if the UR safety configuration was loaded (B-19) | job/`programs/<rev>/verify/` |
| D16 | Cell & runtime configuration | Design (cell part: Admin) | all runtime modules, M4 (soft limits) | Cell part in `cell/config/cell.yaml` (Admin-owned, schema in `awm_core`, hashed into the context record): soft limits, clearances, padding, acceptance and path-shaping parameters, positioner range, PLC handshake timeouts, context policy table. Software defaults (solver and service timeouts, retries) in the package share, not hashed. Robot joint limits only from `ur_description` / D14. No credentials (B-08) | `cell/config/` (cell part), `awm_bringup/config/` (defaults) |
| D17 | Run recording | M11 | M12 | rosbag2 (joints, I/O, status) | `logs/bags/<date>_<job>/` |
| D18 | Part weld report | CLI export (Phase 1), M11 (Phase 2) | M12, customer | CSV (Phase 1); PDF (Phase 2) | job/`reports/` |
| D19 | Part placement & collision scene | M1 (A15, A17) | M4, M7 | `placement.schema.json` (incl. `source` = `study` / `fixture_cad` and `verified` — false until REF_* markers (OI-23) and A52 / A53 measurements exist; only simulation scope accepts false, B-12) + part, fixture and clamps as attached collision objects on `positioner_flange` + scene manifest (objects, parents, transforms, mesh references, units, scene hash; B-07) + meshes | job/`placement.json`, job/`cache/` |
| D20 | 3D view model | M1 (A13) | UI (S2, S4), M2 | `view.glb` + `topology.json` | job/`cache/view/` |
| D21 | Cell & fixture CAD | Mechanical design (Fusion) | M1 (A15, A18) | STEP AP214 / AP242 assembly, mm, marker components `REF_*` | cell/`cad/` (cell), job/`fixture.step` (fixture) |

![Figure 3 – Data flow between modules](APLSMN601-DES-001_figures/fig3_dataflow.png)

### 5.2 Job package

```
<repo_root>/
├── cell/
│   ├── wps/                      D05  WPS-<id>.json
│   ├── calibration/              D08  <date>_<id>.json, active.json
│   ├── config/cell.yaml          D16  cell part (hashed, Admin-owned, B-08)
│   ├── cad/                      D21  cell STEP assembly + cad_map.yaml (component → role, up axis)
│   ├── model/                    D14  meshes and frames generated by A18
│   └── io_map.yaml               D11  (generated from DES-002)
├── jobs/
│   └── <PartNo>_<PartRev>/
│       ├── model.step            D01  (copy)
│       ├── part.json             D03  part no., rev, file name, SHA-256
│       ├── seams.json            D04
│       ├── process.json          D06
│       ├── placement.json        D19  fixture -> part, orientation, scene objects, source
│       ├── fixture.step          D21  fixture assembly (source = fixture_cad)
│       ├── cache/                D02  model.brep, collision.stl (part collision mesh with face_id)
│       ├── cache/view/           D20  view.glb + topology.json
│       ├── path.json             D07
│       ├── programs/<prog_rev>/
│       │   ├── program.json      D09
│       │   ├── verify/           D15
│       │   ├── bundle/           frozen copies at release (ADR-20)
│       │   └── release.json      D10
│       ├── active_simulation.json    pointer to the released revision (scope simulation)
│       ├── active_production.json    pointer to the released revision (scope production)
│       ├── runs/<run_id>/        planning-run folders (ADR-19)
│       ├── inspections/          D13
│       └── reports/              D18
└── logs/
    ├── weld_log.sqlite           D12
    └── bags/                     D17  rosbag2 per run
```

Rules:

- Every JSON file has `schema_version`, `created_by`, `created_at` and `inputs` (list of input file names with SHA-256). A module refuses inputs whose hash does not match.
- Seam IDs (`S01`, `S02` …) are assigned by the user and kept stable. Each seam stores a geometric fingerprint of its B-Rep edges so it can be re-matched when the STEP revision changes. The seam geometry is self-contained in D04 and the release bundle (B-09). R3 (with A93): tolerance match (Hausdorff ≤ 0.05 mm, same curve type and solid pair); the Engineer confirms every re-matched seam; the re-match report goes into the bundle (B-25).
- A released program is read-only. Any change (seam, parameter, path, calibration) creates a new program revision that must be verified and released again (A93 → A2–A4). Phase 1: a calibration change triggers an automatic full re-plan and re-verify of all jobs; requalification rules per OI-36 (B-25).
- The repository root is a parameter. Phase 1: a local folder on the IPC, backed up to the shared drive. Phase 2: database / MES if needed.
- A `production` release requires `placement.json` with `source: fixture_cad` and `verified: true` (ADR-17, B-12).
- **Schema versions (CP-29 as modified by B-09).** `schema_version` is `MAJOR.MINOR` and atomic writes (temporary file + rename) are existing behaviour in `awm_core`. A MINOR change only adds optional fields; a module refuses an unknown MAJOR version. Released bundles are never migrated in place. Migration scripts are mandatory only after the first job created by an Engineer; before that, test jobs are regenerated. A MAJOR change of D09 / D10 needs scripted regeneration and re-verification of the active releases before the runtime update.
- **One writer per job (B-11).** The active planning run is the job lock; `awm_api` refuses edits while it runs. No lock files.
- **Release bundle (ADR-20, B-09).** At release M7 copies D03, D04 + seam geometry, D06, D07, D19 with the scene manifest, the referenced D05, D15, the records and the cell part of D16 into `programs/<rev>/bundle/`, writes `release.json` with the SHA-256 of each copy and the context record, and locks the folder. The job level keeps one active pointer per scope (`active_simulation.json`, `active_production.json`); a simulation release never displaces a production one. The runtime checks the active pointer, the frozen manifest, the D09 SHA-256 and the context record. Job-root files stay editable for the next revision. HW-2 gate: states superseded / revoked, Admin revocation with a reason, append-only `releases.log`.
- **Context record (B-10).** `awm_core` builds one context record: cell id / rev, D14 hash, cell-config hash, D08 and component hashes, robot serial or `SIM`, kinematics hash, controller calibration and UR safety checksums, `ur_installation` block, Git commit with dirty flag, image digest. Later fields are MINOR additions; the Sim stages fill what exists. D07, D09 and D10 embed it. One compare function and a policy table in `cell.yaml` decide refuse / re-plan / warn per field.

### 5.3 Coordinate frames

![Figure 4 – Frame chain](APLSMN601-DES-001_figures/fig4_frames.png)

| Frame | Defined by | Stored in |
|---|---|---|
| `world` | Cell origin on the Pismo frame | D14 |
| `base` | UR5e base | D14 (nominal), D08 (measured if needed) |
| `fixture` | Fixture reference points (3-point method, A52) | D08 |
| `part` | STEP model origin; nominal pose on fixture from fixture design | D19 nominal (B-07); A53 result (go / no-go in Phase 1, B-21) in D08 |
| `tcp_laser` | On the beam axis at the nozzle / wire tip, i.e. the contact point on the seam; tool Z = beam direction; `focus_offset_mm` (D06, default 0) moves the planned point along the beam axis. Assumption until IPG data (OI-2); `tcp_mig` reserved for Phase 2 | D08, one TCP component per tip type (B-25); written to the UR installation together with payload / CoG / mounting (`ur_installation` block in the context record, checked at INIT where the driver exposes it; HW-1) |
| `positioner_axis`, `positioner_flange` | Rotary table axis (A55) and the flange at angle θ; `fixture` and `part` are children of `positioner_flange` | D08 (nominal from D21) |

**Axis convention (decision 2026-10-27).** Z is up in `world` and `base`. Seam frame: X = travel direction, Z = bisector of the face normals pointing out of the material, Y = Z × X; the beam axis is −Z. CAD files may use another up axis (the Fusion cell CAD is Y-up); A18 applies a fixed rotation per file, stated in the CAD map.

**Nominal frames from CAD (ADR-16).** `world`, `base`, `positioner_axis`, `fixture`, `part`, `tool0` and `tcp_laser` are read from marker components `REF_WORLD`, `REF_UR_BASE` (aligned with the UR controller `base` frame, which is `base_link` rotated 180° about Z in `ur_description`), `REF_POSITIONER`, `REF_FIXTURE`, `REF_PART`, `REF_TOOL0`, `REF_TCP`.

D07 is always in the `part` frame. M6 computes `base_T_tcp = base_T_axis · Rz(θ) · flange_T_fixture · fixture_T_part · part_T_pose · tcp⁻¹` with the active D08 (θ = positioner angle of the point or segment) and writes the context record (with the D08 hash) into D09. The runtime refuses a program whose context record differs from the active context where the policy table says refuse — always for D08, D14 and the cell configuration (fixes F14; B-10).

**Robot kinematics (ADR-18, B-21).** The nominal `ur_description` kinematics can differ from the real robot by centimetres (Fact as a bound: `ur_calibration` README [R14]). Calibrated kinematics are mandatory for every motion on hardware, the dry run included: the factory-calibrated kinematics of the installed UR5e are extracted with `ur_calibration` and used in D14 for planning, FK checks and the program. Planning calls IK only through KinematicsBackend; before HW-1 the analytic seeds are refined by damped least squares on the D14 chain to ≤ 0.01 mm / 0.01° within the seed's configuration label, and the M6 FK acceptance tightens from 0.2 mm to 0.01 mm (tested in Sim-1 with a perturbed kinematics file). The kinematics hash and the controller calibration checksum are part of the context record. The `ur-analytic-ik` version is pinned. In simulation the nominal kinematics are used and marked `NOMINAL`.

**Part frame (B-21).** `fixture_T_part` = nominal (D19). A53 in Phase 1 is a go / no-go check; LOCATE_PART returns a typed result (go / no-go or measured offset; no-op provider in simulation); D09 reserves a `touch` segment type. No runtime per-part correction in Phase 1 (design rule 1); any correction is an explicit exception decided by Truong Hoang (OI-43, escalation E-07). The touch-check method is decided at the HW-1 gate with measured E2, OI-16 and OI-30.

### 5.4 Collision model — data sources

Collision checking (FCL in MoveIt2) works on triangle meshes and primitives, never on B-Rep. STEP stays the source of truth; meshes are generated from it with controlled tolerance.

| Object | Source file | Collision representation | Notes |
|---|---|---|---|
| Part | Customer STEP (D01) | Mesh from M1, chord ≤ 0.1 mm near seams, coarser elsewhere; each triangle keeps its `face_id` | No ACM exemption (B-04). The nozzle collision geometry stops at a contact standoff s_c (D16) before the contact point. Contact is checked explicitly: contact point(s) on the seam faces (face_id of n1 / n2) within tolerance, no other face within s_c. Pre-load per OI-22; with a rigid tip at 0 mm M7 marks 'contact not assured' and A73 must prove contact. Part, fixture and holders rotate with the positioner |
| Fixture, clamps | Fusion CAD (D21) | Simplified STL / convex hulls in D19 only, attached to `positioner_flange` (B-07) | 3 mm padding |
| LightWELD head + mount, fiber and wire-feed conduit near the head | Head and mount from the Fusion cell CAD (D21; the head is a surface model) | Convex hulls; nozzle collision geometry follows the tip shape (Phase 1: Ø 20 mm nozzle of TST-002 §3.1 until IPG CAD) and stops at the contact standoff, `nozzle_body` padded; conduit as capsules; wire guide as a link that turns with ψ once OI-15 gives its geometry (B-04, B-05) | Fiber bend radius ≥ 50 mm checked by A48 |
| UR5e | ur_description | Vendor collision meshes | — |
| Pismo frame, table | Fusion CAD (D21) | Boxes / convex hulls | — |
| Positioner (rotating holders, shaft side) | Fusion CAD (D21) | Convex hulls moving with `positioner_joint` | 3 mm padding; Mode C checked over the full turn |

No additional file type is needed from the customer.

**Clearance values (D16).** `padding_mm` = 3 (every head link; applies to the validity check), `weld_clearance_mm` = 5 (head body, mount and conduit during weld segments; TST-002 C2), `air_clearance_mm` = 10 (approach, retract, air and index moves; TST-002 C4), `contact_standoff_mm` (s_c) = 0.3 (Assumption, B-04). Weld and air clearances are distances on **unpadded** geometry, measured by distance query or by inflating the head / mount / conduit links by exactly the clearance in a separate validity check; the method is recorded (B-03). Every segment is checked densely, not only at trajectory waypoints: the step is chosen so that no robot or head point moves more than `max_check_displacement_mm` (< 2 × (guaranteed clearance − margin)) between checked states; positioner steps Δθ ≤ 2·(c − margin) / r_max of the rotating set (B-03).

## 6. Interaction and state viewpoint

### 6.1 Cell state machine (M8 + PLC)

![Figure 5 – Cell state machine](APLSMN601-DES-001_figures/fig5_states.png)

| State | Entry condition | Main action | Exit |
|---|---|---|---|
| INIT | Power-up | Start ROS2 nodes, connect UR, PLC and positioner drive, reference (home) the positioner, load D16; device-identity / mode check (simulation vs hardware drivers; a mix → NOT_READY, B-19) | All links OK → NOT_READY |
| NOT_READY | Link loss, safety not OK, robot not in remote | Show reason on HMI | All permissives OK → IDLE |
| IDLE | Permissives OK | Operator selects released job | Job valid (release + calibration hash) → READY |
| READY | Job loaded, robot and positioner at home | Operator loads and clamps the parts and tacks them by hand (OI-26); wait for cycle start (outside guard, door closed) | Start → LOCATE_PART |
| LOCATE_PART | Start accepted | A53: typed result (go / no-go or measured offset; no-op provider in simulation, B-21); fixed-jig check in Phase 1, method decided at the HW-1 gate | Part found → WELDING; not found → FAULT |
| WELDING | Part located | Seam loop (6.2) | Last seam done → CYCLE_END; pause request → PAUSED (Phase 2); any error → FAULT |
| PAUSED (Phase 2) | Operator pause | Stop in place, emission off, retract (A86) | Resume → back up a few mm along the seam, restart weld sequence → WELDING; stop → FAULT |
| CYCLE_END | All seams done | Go home, close weld log, PLC end-of-cycle | Part unloaded → READY |
| FAULT | Any alarm | Emission off, wire feed off, controlled stop of robot and positioner, alarm text | Operator reset + cause cleared + home / start-state check → IDLE; Phase 1 recovery is manual with the teach pendant; interrupted seam → NCR draft (B-19); reverse replay Phase 2 (A86) |
| E_STOP | Safety chain open (hardware) | Robot and power stopped by hardware | Safety reset → NOT_READY |

The PLC keeps its own mirror of the cycle state and can force FAULT at any time (LightWELD fault, positioner drive fault, interlock open, door open, heartbeat loss from IPC). Heartbeat loss acts as the watchdog: the PLC switches emission off without waiting for the IPC. **Emission follows the welding axis (Proposed, B-17; PLC requirement for DES-002, implemented by the human PLC author at HW-2).** Emission while the welding axis (robot or positioner, `weld_axis` in WeldStart) is stationary is allowed only inside the start-dwell window; after motion has started, a stop ends emission at once (same sequence as WeldEnd); in Mode C, robot motion during emission ends emission; total emission is capped at the expected duration + margin (D16). Inputs: hard-wired UR safety output 'Robot Moving' and drive in-motion output (7.2). The heartbeat counter is advanced by M8's state tick and only forwarded by M10; on loss the PLC switches emission off first, then stops the positioner.

### 6.2 Seam execution (weld events)

![Figure 6 – One seam: sequence](APLSMN601-DES-001_figures/fig6_sequence.png)

Design rule: **process events happen only while the robot is at rest at segment boundaries** (approach end, weld end). This removes the need for hard real-time coupling between ROS2 trajectory execution and laser switching in Phase 1, and works for both MIG and laser:

1. M9 moves to the approach pose, then the weld start pose (stored trajectory segments).
2. M8 → M10 → PLC: `WeldStart(seam_id, wps_id, preset)`. PLC checks interlocks (door, E-stop, external interlock loops, workpiece-clamp contact), selects the LightWELD preset, gas pre-flow, wire-feed ready, then trigger → emission.
3. PLC → M10 → M8: `LaserReady` / emission-on feedback within the timeout from D06; on timeout no automatic re-trigger: EmissionOff is confirmed, then FAULT (B-17). EmissionLost during a segment → FAULT and an interrupted-seam record. Dwell time from D06.
4. M9 executes the weld segment at constant TCP speed (time-parameterised by M4 from the WPS travel speed; fixes F12); M9 sends the D09 points verbatim (B-15).
5. At weld end: `WeldEnd` → power ramp-down / dwell (if the preset supports it), emission off, wire-feed stop / retract, post-flow; PLC confirms `EmissionOff`. Closed 360° seams end with the overlap planned in A42.
6. M9 retracts and moves to the next seam (collision-checked air move).

**Mode C (positioner weld, seam coaxial with the positioner axis).** Steps 1–3 as above with the robot at the weld start pose and the positioner at rest; step 4 is a positioner weld segment executed by M9 (robot at rest, positioner turns at ω = travel speed / seam radius plus the closing overlap; ramps planned by A47 and placed inside the dwell or the closing overlap, B-20); step 5 starts when the positioner is at rest again. **Mode B:** the positioner index move to the planned angle is made with emission off, before step 1. Rule: WeldStart and WeldEnd only while robot **and** positioner are at rest.

Speed-scaling rule: `scaled_joint_trajectory_controller` slows the robot when the UR speed slider or safety scaling is below 100 %, which changes the travel speed and the heat input. M9 monitors the speed-scaling value; weld segments start only at 100 % and any change during a weld segment → `WeldEnd` + FAULT. Air moves may run with override. Exception (Proposed, B-16, pending OI-38): a `trial`-scope dry run with emission disabled by hardware may run weld segments with override. Weld segments are planned clear of UR safety slow-down zones and reduced-mode triggers (B-25, HW-1). The same rule applies to a Mode C positioner weld segment: it starts only at 100 % (no override on weld segments). In Phase 1 the robot and the positioner never move together during emission, so no common scaling is needed; continuous 7-axis motion (Phase 2) must apply one scaling to both or stop (ADR-11).

LightWELD faults, wire-feeder faults, gas faults, positioner drive faults (hard-wired ready / fault) and interlock openings are handled by the PLC (emission off) and raised to M8 as FAULT (A84). Exact LightWELD status signals depend on the IPG automation interface (OI-2).

Phase 2 option: if a flying start or a laser power ramp along the path is needed, move the event timing into the UR controller (URScript I/O at waypoints) — see ADR-01.

### 6.3 Positioner synchronisation (Phase 1 Modes B / C, Phase 2 continuous motion, ADR-11)

**Fact.** The UR5e (PolyScope 5) has no external-axis support: unlike KUKA or Fanuc controllers it cannot interpolate a 7th axis. `ur_robot_driver` streams joint set-points at 500 Hz through the External Control URCap. `scaled_joint_trajectory_controller` slows trajectory progress with one speed-scaling value. `passthrough_trajectory_controller` hands the whole trajectory to the UR controller and therefore cannot include a 7th joint. Synchronisation must be done in ROS2.

| Option | Method | Main error source | Assessment |
|---|---|---|---|
| A1 | One `controller_manager`, two hardware components (UR + positioner servo), one joint trajectory controller for 7 joints at 500 Hz, PREEMPT_RT kernel | Cycle jitter + UR servo look-ahead delay | Standard ros2_control pattern. **Interpretation:** if the positioner joint is in the same scaled controller, the speed-scaling value also slows the positioner — to be verified in the spike |
| A2 | Robot follows the measured positioner angle (electronic cam): positioner turns at constant ω (PLC / NC); a custom controller reads θ and interpolates robot set-points as a function of θ | Encoder read latency × ω | Most robust against lag and speed scaling; custom controller needed |
| B | Rotate – stop – weld (fixed table angle per arc segment) | None (no simultaneous motion) | Simple; joints between segments. **Phase 1 (Mode B)** |
| C | Coaxial seams (pipe-to-flange): robot static, only the positioner turns | None | Covers most round seams. **Phase 1 (Mode C)** |

**Estimate (Assumption, to be measured).** Position error ≈ travel speed × timing error. At 20 mm/s: 2 ms jitter (RT kernel) gives about 0.04 mm; 10 ms (non-RT) gives about 0.2 mm. A constant lag can be calibrated out; only jitter remains. Against the laser band of 0.1–0.2 mm this is feasible with an RT kernel but must be proven.

**Positioner drive (decision 2026-10-27: ROS 2 controls the positioner).** The drive is a ros2_control hardware component (`awm_positioner`) in the same controller manager as the UR driver. Proposed: closed-loop drive with encoder on EtherCAT (CiA 402, `ethercat_driver_ros2`) on its own IPC port, PREEMPT_RT kernel (OI-14). This serves Modes B / C now and keeps Option A1 open for Phase 2. A drive behind the Beckhoff PLC over ADS was rejected: not deterministic, and it would block A1. Safety stays in hardware: drive STO from the safety chain; drive ready / fault hard-wired to the PLC, which switches emission off on a drive fault.

**Phase 1 decision (2026-10-27).** The mechanical design contains a 1-axis positioner (motor, coupling, shaft, two timing-belt stages, holders on the jig; horizontal axis). It is part of Phase 1 (OI-20 closed). On the golden part 564082, joint J1 (pipe nipple ↔ solid) is coaxial with the axis → Mode C; J2 (solid ↔ elbow) is 76.6 mm off the axis with its axis perpendicular to it → robot alone or Mode B. Mode B index moves approach the target angle from one direction (removes belt backlash from E7, B-20). Accuracy: an angular error δ gives r·δ at the seam; at r = 13.3 mm, 0.1 mm ⇔ 0.43°, so backlash and encoder resolution must be checked (OI-25).

## 7. Interface viewpoint

### 7.1 Software interfaces (ROS2)

| Interface | Type | Provider → user | Content |
|---|---|---|---|
| `/awm/cell/command` | service | M8 ← HMI | select_job, start (request only — physical start button required, 7.3), stop, reset, home; pause / resume in Phase 2 |
| `/awm/cell/state` | topic (latched) | M8 → HMI, M11 | state, job, seam, progress, active alarms |
| `/awm/execute_segment` | action | M9 ← M8 | program rev, release SHA-256, segment id and segment controller (robot, positioner, weld or index segment), speed override (air / index moves only); feedback %, result (B-15, B-17) |
| `/awm/weld_event` | action | M10 ← M8 | WeldStart(seam, WPS id / rev, preset, segment type, weld_axis, dwell_s, expected_duration_s, ω, release SHA) / WeldEnd; results LaserReady / EmissionOff / EmissionLost (with time) / fault; typed in `awm_interfaces` v1, behaviour per the DES-002 weld-event table (B-17, B-18) |
| `/awm/plc/status` | topic | M10 → M8, M11, HMI | PLC state, permissives, LightWELD status (ready, emission, fault, interlocks), wire feeder, gas, active preset |
| `/awm/alarm` | topic | all → M8, M11, HMI | alarm id, source, severity, text EN / VN |
| MoveIt2 `compute_ik`, `compute_cartesian_path`, `check_state_validity` | services | move_group → M4 | planning (Workbench graph only): OMPL air moves; validity / IK through the M4 backends (B-06) |
| ur_robot_driver `scaled_joint_trajectory_controller`, `io_and_status_controller`, `speed_scaling_state_broadcaster`, dashboard | actions / services / topics | UR driver → M9, M5 | motion, speed-scaling value, program state, robot mode (topic names to be confirmed on the installed driver version) |
| `positioner_trajectory_controller/follow_joint_trajectory` | action | `awm_positioner` controller → M9 | positioner segments (joint `positioner_joint`); state in `/joint_states` |

### 7.2 Hardware / field interfaces (summary — detail in DES-002)

![Figure 7 – Deployment and network](APLSMN601-DES-001_figures/fig7_deployment.png)

| Link | Protocol | Signals (examples) | Time-critical? |
|---|---|---|---|
| IPC ↔ UR5e | ur_robot_driver (RTDE + External Control URCap), Ethernet | joint trajectory, robot mode, I/O read | Motion: yes (500 Hz, handled by driver) |
| IPC ↔ PLC | ADS / TCP (pyads) with notifications | commands, weld parameters, status, alarms, heartbeat 100 ms | No (events at rest) |
| UR5e ↔ PLC | Hardwired digital I/O (Phase 1) — PROFINET / Modbus TCP option | robot running, at home, program fault, PLC permissive | Medium |
| PLC ↔ IPG LightWELD 1000 | Digital I/O on the 12-pin customer interface (external interlocks, trigger / pedal, wire feeder) + Ethernet (web interface, parameter / preset set-up); analog power set or fieldbus only if the IPG automation interface provides it (OI-2, OI-19) | trigger / emission, preset select, ready, fault, interlock A / B loops, workpiece-clamp contact | Yes (PLC) |
| PLC ↔ MIG welder | Phase 2 | — | — |
| IPC ↔ positioner drive | EtherCAT (`ethercat_driver_ros2`) or CANopen CiA 402 (`canopen_402_driver`); every command derived from the D09 segment (ADR-22, Proposed; OI-14) | position set-points, actual position, drive status | Yes (ros2_control cycle, PREEMPT_RT) |
| Positioner drive ↔ PLC / safety | Hard-wired | drive ready / fault to the PLC; STO from the safety chain | Safety function, independent of software |
| Safety chain | Hardwired safety relay / TwinSAFE ↔ UR safety I/O | E-stop, guard door, laser interlock, robot safeguard stop | Safety function, independent of software |
| UR5e → PLC (Robot Moving) | Hard-wired UR safety output (Proposed, B-17; DES-002, HW-2) | Robot Moving: emission tied to welding-axis motion | Yes (PLC) |
| Positioner drive → PLC (in motion) | Hard-wired (Proposed, B-17; DES-002, HW-2) | drive in-motion | Yes (PLC) |
| Rotary ground contact on the positioner output shaft | Mechanical / electrical (open design input, Nghia Nguyen; B-17) | LightWELD part-contact circuit in Mode C | — |

Ownership rule: the PLC is the only device that switches the LightWELD. The IPC requests; the PLC decides based on interlocks. The positioner is moved only by ROS 2 (M9); its safe stop is hardware (STO).

### 7.3 User roles and command security

| Role | Allowed |
|---|---|
| Operator | Select and run released jobs; load, clamp and tack parts; start / stop / reset; view status and alarms |
| Engineer | Create and edit jobs, pick and edit seams (only this role, decision 2026-10-27), placement study, run simulation and dry run, request release |
| Admin | Calibration, cell configuration D16, cell model import (A18), user management, approve release |

Rules:

1. Every motion or process command (start, home, weld; no jog from any UI, B-13) goes through `awm_api` and M8, which check role, cell state and permissives. A UI never talks to the robot or PLC directly (no rosbridge for commands).
2. A Start from any UI is a request; motion starts only after the physical start button at the cell (outside the guard). E-stop is always hardware.
3. Status to the UI is pushed at 10–30 Hz over WebSocket — display only, not used for control.
4. Identities in records come from the login only (B-24). Credentials are kept in a git-ignored secrets file with salted hashes, never in D16 (B-13). A CLI identity is acceptable for simulation scope only.

### 7.4 Web API (`awm_api`, first version)

Rules (CP-32 as modified by B-13): all paths are under `/api/v1`; the OpenAPI document generated by FastAPI is the contract and is reviewed like `awm_interfaces`; errors use one model (RFC 9457 `application/problem+json`: type, title, status, detail, plus `job_id` / `seam_id` where relevant); long operations (plan, import, verify) return `202 Accepted` with a run id and report progress over the WebSocket. The Stage column gives the first stage of each endpoint; only the Sim-1 subset is built in Sim-1; functions without an endpoint in their stage run through the CLI.

| Endpoint | Method | Purpose | Role | Stage |
|---|---|---|---|---|
| `/auth/login` | POST | Login, token with role | All | Sim-1 |
| `/jobs` · `/jobs/{id}` | GET / POST | Job list, create from STEP upload, job detail and revisions | Engineer | Sim-1 |
| `/jobs/{id}/view` | GET | D20 view model (`view.glb`, `topology.json`) | All | Sim-1 |
| `/jobs/{id}/seams` | GET / PUT | Seam list D04 (picked edge IDs + attributes, mm) | Engineer | Sim-1 |
| `/jobs/{id}/process` | GET / PUT | Process assignment D06 | Engineer | After Sim-1 (CLI before) |
| `/jobs/{id}/placement` | GET / PUT | D19 placement: candidates (`study`) or import from the fixture CAD (`fixture_cad`) | Engineer | Sim-1 (GET); PUT after Sim-1 |
| `/jobs/{id}/plan` | POST | Request planning (M4) — returns `202` + `run_id` (one run per cell, `409` when busy), progress over WebSocket (ADR-19) | Engineer | Sim-1 |
| `/jobs/{id}/runs/{run_id}` | GET / DELETE | Planning run: status and result; cancel (SIGTERM of the run subprocess) | Engineer | Sim-1 |
| `/jobs/{id}/simulation` | GET | Trajectory samples + per-point status for playback | Engineer | Sim-1 |
| `/jobs/{id}/release` | POST | Release request / approval (M7), scope `simulation` or `production`; the release CLI / API refuses `production` until the HW-2 gate code exists (B-16) | Engineer / Admin | Sim-1 (simulation scope) |
| `/wps` | GET / POST | WPS / LightWELD preset library D05 | Welding engineer | After Sim-1 (CLI before) |
| `/cell/command` | POST | select_job, start request, stop, reset (→ M8); no jog from any UI; home only as a stored segment with the physical start button (B-13) | Operator | HW-1 (S6) |
| `/calibration` | GET / POST | Calibration runs and records D08 | Admin | HW-1 (CLI before) |
| `/cell/model` | GET / POST | Cell CAD import (D21 → D14), marker check report | Admin | After Sim-1 (CLI before) |
| `/reports/{part}` | GET | Part report D18 | All | HW-2 (CSV export before) |
| `/users` | GET / POST | Users and roles | Admin | Phase 2 (fixed local accounts before) |
| `ws://…/status` | WebSocket | Cell state, seam progress, LightWELD status, alarms (10–30 Hz) | All | Sim-1 (run progress); cell state HW-1 |

## 8. Deployment viewpoint

| Node | Hardware (proposal) | Software |
|---|---|---|
| IPC | Industrial PC, 8+ cores, 32 GB RAM, NVIDIA GPU optional, 3 × Ethernet (cell LAN, office, EtherCAT to the positioner drive) | Ubuntu 24.04 with PREEMPT_RT (real-time) kernel as recommended for ur_robot_driver, ROS 2 Jazzy (ADR-10), MoveIt2, ur_robot_driver, `awm_api` (FastAPI) + static `awm_web` build behind a reverse proxy, M1 CAD process with pythonocc-core in its own conda environment (ADR-15); Docker image for reproducible build; runtime image from the public `ros:jazzy` base, non-root, pinned by digest (HW-1 entry; Proposed, OI-42, B-23) |
| Operator panel / engineering PC | Panel PC or tablet at the cell; any PC on the cell LAN | Chromium / Chrome in kiosk mode for S6; normal browser for engineering screens |
| UR5e controller | e-Series | PolyScope 5.x [TBD], External Control URCap, safety configuration, TCP from D08 |
| Cell PLC | Beckhoff CX / IPC with TwinCAT 3 [TBD] | `APLSMN601_CellPLC`, EtherCAT I/O, TwinSAFE or external safety relay |
| Development | Engineer laptop, WSL2 (host Ubuntu 22.04 left unchanged) | ROS 2 Jazzy inside the Docker image of the Tac demo (same image as the IPC); URSim container for simulation (`sim_ursim` launch; `sim_mock` for Sim-1); CI: self-hosted runner (never on the cell LAN) with the image built from `docker/Dockerfile` and the conda OCC environment mounted read-only; unit, contract and planner regression per merge, `sim_mock` integration nightly; until the runner exists, `scripts/ci` in `awm_dev` before each merge, logged in PROGRESS.md; demo image for development only (B-23) |

Network (proposal, to be fixed in DES-002): isolated cell LAN 192.168.10.0/24 — IPC .10, UR5e .20, PLC .30, laptop .100. The IPC second port connects to the office network for Git and the shared drive only; the third port is the EtherCAT segment of the positioner drive.

### 8.1 Network and host security (Proposed, B-23; implementation is a HW-1 entry item)

| Item | Rule |
|---|---|
| Zones | Cell LAN and office network; the IPC does not route (ip_forward = 0); default-deny host firewall |
| Office port | Outbound Git / shared drive only; HTTPS in only if Engineer screens are needed there |
| Cell port | UR (RTDE, dashboard, reverse interface from the robot IP), one ADS route to the PLC, HTTPS from the panel |
| ROS | Two ROS domains on the IPC: engineering graph (`move_group`, planning) and cell graph (no MoveIt); LOCALHOST discovery; no WAN DDS profile |
| Image | Runtime image from the public `ros:jazzy` base with an apt snapshot and a micromamba lock for pythonocc-core 8.0.1; non-root, not privileged, only CAP_SYS_NICE / rtprio (plus EtherCAT access if chosen); pinned by digest; the demo image never goes to the IPC (OI-42) |
| Devices | UR inbound access restricted to the IPC and unused services disabled; one ADS route; LightWELD web interface not reachable in production; the real ADS client is not installed in the Sim-stage image |
| Web | HTTPS only, session expiry, lockout, append-only audit log (login, release request / approval, calibration activation, D16 / WPS / user changes) |
| CI | Self-hosted runner never on the cell LAN; the IPC never runs CI jobs |

## 9. Architecture decisions

| ADR | Decision | Status | Alternatives / reason | Decide by (B-14) |
|---|---|---|---|---|
| ADR-01 | **Motion execution through ROS2**: M6 stores pre-planned, time-parameterised joint trajectory segments; M9 replays them with ur_robot_driver. URScript export (Remote TCP / Toolpath URCap) kept as fallback (WBS 4.3). | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; review at the Sim-2 exit; B-14) | URScript `movel` / `movep` gives native constant TCP speed and runs if the IPC stops, but duplicates planning logic and is harder to simulate with MoveIt. ROS2 reuses the Tac demo and the simulation chain. | 2026-11-06 |
| ADR-02 | **B-Rep CAD kernel OCCT (pythonocc-core)**; STL only for display / collision meshes | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; B-14) | Mesh pipeline of the demo loses exact edges and needs hard-coded seams (F03, F04) | 2026-11-06 |
| ADR-03 | **Manual seam selection only**: seams are picked edge by edge in the 3D simulator view, once per part number, then simulated and released. Automatic seam detection is removed from scope | Confirmed 2026-10-22 (replaces "automatic detection in Phase 2") | Scope control (charter risk 04); the people who know the weld symbols choose the seams | Decided |
| ADR-04 | **PLC owns the IPG LightWELD 1000 and the laser interlocks** (trigger, preset, interlock loops, wire feeder); IPC ↔ PLC over ADS; process events only at segment boundaries. Weld sequence and watchdog of [R9] 5.7 are implemented in the PLC / M10 | Confirmed for Rev 0.3 (2026-10-22) | [R9] proposes a ROS2 `weld_controller` node driving the welder over UR I/O or fieldbus. Rejected for Phase 1: bypasses PLC interlocks and the Class 4 laser interlock chain; the timing need it addresses is removed by events at rest | Decided |
| ADR-05 | **Plan offline, replay online** — no weld-path planning at runtime; runtime checks only start state and hashes | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; B-14) | Demo plans at runtime and accepts 35 % paths (F01) | 2026-11-06 |
| ADR-06 | **One robot (UR5e) in the Phase 1 cell model** — remove GP25, rail and gripper from D14 and sequencer; the part sits on the fixture of the 1-axis positioner (ADR-11) | Confirmed for Rev 0.3 (2026-10-22); updated 2026-10-27 | Fixed Pismo frame in Phase 1 (F10); keep the demo dual-robot code in a Phase 2 branch | Decided |
| ADR-07 | **File-based job repository**; weld log as JSON lines in Phase 1, SQLite in Phase 2 (B-24) | Repository: to be confirmed as Decided on 2026-11-06 (B-14); weld log: Proposed | Simple, versionable, enough for Phase 1; database / MES in Phase 2 | 2026-11-06 |
| ADR-08 | **Python for all Phase 1 nodes including M9** (the real-time loop is in ros2_control); C++ only on measured need | Proposed (B-19); depends on the PLC requirement 'emission follows the welding axis' (B-17) | Python is fastest for OCCT / geometry work; M9 only sends stored segments and monitors; the demo welder_action_server mixes planning and execution (F01, F12) | Sim-1 exit (before WP4) |
| ADR-09 | **Web UI for Workbench and Operator HMI**: React + TypeScript + react-three-fiber + urdf-loader front-end (`awm_web`), FastAPI + rclpy backend (`awm_api`) on the IPC; design in Section 4.5, API in 7.4 | Decided 2026-10-22 | Seam picking in 3D by engineers and operators, headless Linux IPC, access from panel PC / tablet / customer. Rejected: Qt + OCCT (desktop only, robot simulation needs RViz separately); TwinCAT HMI (far from job and 3D data). Risk: 3D front-end effort — mitigated by picking IDs only (D20) and reusing urdf-loader | Decided |
| ADR-10 | **ROS 2 distribution: Ubuntu 24.04 + ROS 2 Jazzy from Phase 1**, developed inside the Docker image of the Tac demo; host WSL Ubuntu 22.04 stays unchanged | Changed in Rev 0.3 (2026-10-22) — replaces the Rev 0.2 Humble decision | The demo Docker image already runs Jazzy and TST-002 uses it, so no Humble environment is needed. Humble EOL May 2027 vs Jazzy EOL May 2029 (REP 2000); no Humble → Jazzy migration later (OI-9 closed) | Decided |
| ADR-11 | **1-axis rotary positioner in Phase 1, controlled by ROS 2**: `awm_positioner` ros2_control hardware component in the same controller manager as the UR driver, own joint trajectory controller; Phase 1 motion Modes C (robot at rest, positioner turns during emission, coaxial seams) and B (rotate – stop – weld); continuous 7-axis motion (Options A1 / A2) in Phase 2; safety in hardware (STO, drive fault to the PLC). Analysis in Section 6.3 | Decided 2026-10-27 (replaces the Phase 2 slot of 2026-10-22) | The mechanical design already contains the positioner; 360° seams coaxial with its axis need no robot orbit. PLC-driven positioner rejected: not deterministic and it blocks Option A1. Hardware open (OI-14, OI-25) | Decided |
| ADR-12 | **Process-motion planner: MoveIt2 + custom tool-yaw / IK-branch graph search** (Section 4.3), as tested in TST-002. Tesseract + tesseract_planning (Descartes, TrajOpt, OMPL) is the fallback if TST-002 ends in Pivot / No-Go | Proposed — decided at the TST-002 meeting on 2026-11-06 (OI-29). Inputs frozen per TST-002 Rev 0.3 §2 / §5. Tesseract fallback evaluated after Sim-2, not inside Sim-1 (B-01) | pick_ik (Tac demo) returns one IK solution and is not used for the sweep; analytic UR IK gives all 8 branches; Tesseract is widely used in ROS-Industrial welding apps but adds a second planning stack [R9 5.6]. The planner core sits behind KinematicsBackend / SceneBackend, so Tesseract stays a drop-in fallback (B-06). ψ freedom depends on the WPS; TST-002 reports free-ψ and wire-window cases (B-05) | 2026-11-06 (OI-29) |
| ADR-13 | **Simulated drivers behind the same interfaces**: URSim / UR fake hardware for the robot, `sim` mode of M10 for PLC + LightWELD, `mock_components` positioner joint (Phase 1) | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; B-14) | Hardware arrives late (LightWELD on loan, automation interface open, OI-2); lets the full runtime chain run in CI and simulation from day one [R9] | 2026-11-06 |
| ADR-14 | **mm for all project data**: schemas, `awm_interfaces` messages (fields named `*_mm`), UI, reports. One adapter in M0 converts to metres where project code calls third-party ROS packages | Decided 2026-10-22 | Engineers and drawings work in mm. Third-party packages (URDF, TF, MoveIt2, ur_robot_driver) stay in metres per REP 103 and cannot be rescaled safely; risk of unit mistakes is contained by the single adapter and unit tests on it. One adapter per language: `awm_core` (Python) and the `awm_web` units module (TypeScript), both tested on shared fixtures (B-07) | Decided |
| ADR-15 | **CAD module M1 runs as a separate process** with pythonocc-core ≥ 7.9 (conda-forge; tested with 8.0.1 / Python 3.10), exchanging files (STEP in, JSON / glb / mesh out) with `awm_api` and M2 | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; B-14) | pythonocc-core is distributed through conda; keeping it out of the ROS 2 Python environment avoids package conflicts. Same split suits the Web architecture | 2026-11-06 |
| ADR-16 | **Cell, fixture, head and positioner geometry from Fusion STEP assemblies**: read by M1 with OCCT XCAF (component tree, names, placements); frames from marker components `REF_*`; component → role map kept as data (`cell/cad/cad_map.yaml`); fixed CAD → `world` axis rotation per file; the UR5e model always from `ur_description`; no CAD → URDF plug-in; component mapping is default-deny: every STEP component is mapped to a role or to `ignore`, an unmapped component fails the import (B-07) | Decided 2026-10-27; default-deny Proposed (B-07) | The mechanical team designs in Fusion; one source for nominal geometry avoids hand-typed transforms. Fusion construction geometry and joint origins are not exported to STEP, hence marker components. Read test on the first cell CAD: 98 MB in 56 s, names and placements kept | Decided; default-deny 2026-11-06 |
| ADR-17 | **Release scopes**: `simulation` (after a PASS simulation report only; never valid for production) and `production` (PASS report, dry run, trial weld, error budget, placement source `fixture_cad`, Admin approval). The runtime accepts only `simulation` in simulation mode and only `production` with real hardware | Decided 2026-10-27. **Proposed amendment (B-16, pending OI-38 / escalation E-02):** scopes simulation / trial / production. `trial`: hardware mode only, authenticated Engineer; dry run when the PLC reports emission disabled by hardware (override allowed on all segments), trial weld at 100 %; parts marked TRIAL in D12. `production`: promotion of the same frozen bundle (same D09 SHA) after PASS trial records and the A74 gate checks. The runtime accepts `simulation` only in simulation mode, `trial` / `production` only in hardware mode. Before Sim-2 the M8 scope check is a data table; until the decision the release CLI refuses `production` | Sim-2 exercises the same release and hash checks as production without faking A72 / A73 records. Without a trial scope A72 / A73 cannot run through M8 / M9 (production needs their records; A72 at reduced speed conflicts with the 100 % rule) | Decided; amendment per OI-38 |
| ADR-18 | **Accuracy chain**: on hardware, planning and programs use the factory-calibrated kinematics of the installed UR5e (`ur_calibration`), mandatory for every motion including the dry run; IK only through KinematicsBackend with refinement on D14 before HW-1; kinematics hash and controller calibration checksum in the context record; no runtime per-part correction in Phase 1 (design rule 1) — any correction is an explicit exception decided by Truong Hoang (OI-43, E-07); the error budget (12.2) decides the part-location method at the HW-1 gate | Proposed (CP-24 as modified by B-21) | Nominal kinematics may be off by centimetres (Fact as a bound, [R14]); offline programming from CAD depends on absolute accuracy, not repeatability; a runtime IK re-solve would contradict ADR-05 | HW-1 gate |
| ADR-19 | **Planning runs as CLI subprocesses**: `run_id`, run folder, one run slot per cell (`409` when busy), job edits refused during a run, cancel = SIGTERM; weld / approach / retract segments identical for identical inputs within one image digest; air moves stored and re-verified (4.3) | Proposed (CP-28 as modified by B-11) | One Engineer in Phase 1: no queue and no lock files needed; reuses the tested CLIs with crash isolation; a byte-equal D07 cannot pass (headers carry `created_at`, OMPL is random). Long-lived action worker decided at the Sim-1 exit on measured scene-build time | 2026-11-06 |
| ADR-20 | **Immutable per-revision release bundle** with manifest and one active pointer per scope (5.2); the runtime checks the active pointer, the frozen manifest, the D09 SHA-256 and the context record | Proposed (CP-29 as modified by B-09) | A manifest over editable job files breaks the running release when the next revision is edited (observed in the Rev 0.2 chain); `release.json` was one job-level file that a new release overwrote. Alternative: Git-backed job repository (Phase 2 option) | 2026-11-06 |
| ADR-21 | ~~**`awm_api` split**: FastAPI gateway and a separate rclpy bridge node~~ | **Rejected for Phase 1** (board 2026-10-30, CP-30 withdrawn); revisit at HW-1 when S6 needs live state | All three board members: a second process and an undefined internal protocol with no Phase 1 benefit; no Sim-stage endpoint needs live ROS data; replaced by one FastAPI process with CLI subprocesses (ADR-19) | — (rejected) |
| ADR-22 | **Phase 1 positioner interface sized for Modes B / C only**: any drive mode is allowed if `awm_positioner` derives every command from the D09 segment (no profile set by hand on the drive), the mock and M7 use the same generator, M9 checks the encoder angle and the ω ripple during emission against D16 limits (Assumption until OI-16), and drive in-motion / fault are hard-wired to the PLC (STO from the safety chain). EtherCAT or CANopen CiA 402. EtherCAT with Option A1 stays the Phase 2 target. Detail of ADR-11 — ROS 2 control of the positioner is unchanged | Proposed (CP-33 as modified by B-20) — decide with OI-14 by 2026-11-20 | What is verified (D09 with planned ramps) is what the axis does, while the fieldbus stays free; the interface rule constrains the drive purchase, so it comes first | 2026-11-20 (with OI-14) |
| ADR-23 | **Human review gate**: named human reviewers (CODEOWNERS, protected branch on the company remote) for M8, M9, M10, `awm_positioner`, `awm_interfaces`, the cell part of D16, the A18 / D14 generator including the contact model, the M4 acceptance rules and the M7 checks; the PLC program has a named human author; the software agent prepares, a human approves the merge | Proposed (CP-35 as modified by B-18) — effective 2026-11-23 (Sim-2 start); reviewers named by 2026-11-20 (OI-32); during Sim-1 Truong Hoang signs off verification-defining diffs; prerequisite: protected branch on the company remote (B-23) | All functions are owned by the software agent (OI-28); a Class 4 laser cell needs an independent check of every change that can move axes, request emission or change what counts as verified | 2026-11-20 (OI-32) |

## 10. Reuse of the Tac Huynh ROS2 demo

| Demo package | Decision | Becomes | Changes needed (DES-004 finding) |
|---|---|---|---|
| hold_and_weld_planning — OCCT loader, weld_planner, seam primitives | Reuse, refactor | `awm_cad`, `awm_path` | Install pythonocc in Docker (F04); remove assembly mode and hard-coded pipe seams (F03); poses in part frame |
| hold_and_weld_planning — mesh / CGAL seam extractor | Do not reuse | — | Automatic seam detection removed from scope (ADR-03) |
| hold_and_weld_application — welder_action_server | Reuse core, refactor | `awm_executor` + planning part into `awm_path` | Split planning (offline) from execution; Cartesian threshold 100 % (F01); remove unchecked fallbacks, collision ACM for torch only (F07); config from D16 (F08) |
| hold_and_weld_application — dual_robot_coordinator | Replace | `awm_cell` | New state machine; seam IDs from program, not code (F02); PLC handshake |
| hold_and_weld_application — gripper server, gripper_sampler | Park (Phase 2 branch) | — | F10 |
| hold_and_weld_description | Reuse, simplify | `awm_description` | UR5e + LightWELD head + Pismo frame + fixture only; real head model and TCP from calibration (A51) |
| hold_and_weld_bringup | Reuse, extend | `awm_bringup` | Add `real` launch with ur_robot_driver and URSim (F13); fix dependencies ur_description, pick_ik (F05); no absolute paths (F06) |
| calibration_tools (47 scripts) | Do not reuse | — | Part-specific tuning; replaced by M4 generic reach search and M5 |
| Not in demo | New | `awm_process`, `awm_calib`, `awm_program` events, `awm_verify`, `awm_plc_bridge`, `awm_logger`, `awm_inspect`, Workbench, HMI, PLC program | F11–F15 |

## 11. Traceability: function → module → release

Releases from the charter: R1 Manual Path MVP · R2 First MIG Weld · R3 Smart Path · R4 Laser + Accuracy · R5 Multi-Product. **Proposed renaming after the laser-only decision (needs a CR on the charter):** R2 First Laser Weld · R4 Accuracy + Touch Sensing.

Stage column (B-14): first delivery stage of each function (2.3); the DES-003 Rev 0.6 'Sim stage' column is to be aligned with it.

| Function | Name | Module | Release | Stage |
|---|---|---|---|---|
| A11 | Load STEP file | M1 | R1 | Sim-1 |
| A12 | Check & heal model | M1 | R1 | Sim-1 |
| A13 | Display 3D model | M1 + UI-W | R1 | Sim-1 |
| A14 | Create part record | M1 | R1 (basic), R5 (library) | Sim-1 |
| A15 | Place part in virtual cell | M1 + UI-W | R1 | Sim-1 |
| A17 | Study part placement | M1 + M4 + UI-W | After Sim-1 (manual), R3 (automatic) | After Sim-1 |
| A18 | Import cell CAD | M1 | Sim-1 scripted import, default-deny mapping; marker checks HW-1 | Sim-1 (scripted) |
| A16 | Load STL (fallback) | M1 | Phase 2 | Phase 2 |
| A21 | Select seam edges | M2 + UI-W | R1 | Sim-1 |
| A22 | Define seam attributes | M2 | R1 | Sim-1 |
| A24 | Export seam list | M2 | R1 | Sim-1 |
| A25 | Generate analytical pipe seam | M2 | R1 (test ground truth) | Sim-1 |
| A31 | Select laser mode per seam | M3 | R2 | Sim-1 (sample WPS) |
| A32 | Look up WPS | M3 | R2 | Sim-1 (sample WPS) |
| A33 | Assign weld parameters | M3 | R2 | Sim-1 (CLI) |
| A34 | Maintain WPS library | M3 | R2 | HW-2 |
| A41 | Discretize seam | M4 | R1 | Sim-1 |
| A42 | Set head orientation, focus & wobble | M4 | R1 (angles), R2 (wobble from preset) | Sim-1 |
| A43 | Add approach / retract moves | M4 | R1 | Sim-1 |
| A44 | Solve IK & check reach | M4 | R1, R3 (automatic) | Sim-1 |
| A45 | Check collisions | M4 | R1, R3 (automatic) | Sim-1 |
| A46 | Generate multi-pass offsets | M4 | Phase 2 | Phase 2 |
| A47 | Time-parameterize at travel speed | M4 | R1 (nominal), R2 (verified on weld) | Sim-1 |
| A48 | Check joint soft limits & cable wrap | M4 | R1 | Sim-1 |
| A49 | Coordinate positioner angle | M4 | R1 (Mode C; Mode B with entered angles), R3 (automatic Mode B) | Sim-1 (Mode C; Mode B entered) |
| A51 | Calibrate TCP | M5 | R2 | HW-1 |
| A52 | Calibrate fixture frame | M5 | R2 | HW-1 |
| A53 | Locate part | M5 | R2 (fixed jig), R4 (touch sensing) | Sim-2 (stub); HW-1 |
| A54 | Check error budget | M5 | R2, R4 | HW-1 |
| A55 | Calibrate positioner axis | M5 | R2 | HW-1 |
| A56 | Verify TCP (new, Proposed, B-25) | M5 | R2 | HW-2 |
| A61 | Transform path to robot frame | M6 | R1 (nominal), R2 (calibrated) | Sim-1 |
| A62 | Generate motion program | M6 | R1, R3 (URScript fallback) | Sim-1 |
| A63 | Insert weld & I/O events | M6 + M10 | R2 | Sim-1 markers; Sim-2 M10 sim |
| A64 | Store & transfer program | M6 | R1 | Sim-1 |
| A71 | Simulate program | M7 | R1 | Sim-1 |
| A72 | Dry run on robot | M7 + M9 | R1 / R2 | HW-1 |
| A73 | Trial weld on coupon | M7 | R2 | HW-2 |
| A74 | Release program | M7 | R2 | Sim-1 (simulation scope); HW-2 (production) |
| A81 | Load part & start cycle | M8 + UI-H + PLC | R2 | Sim-2 (minimal) |
| A82 | PLC – robot – LightWELD handshake | M8 + M10 + PLC | R2 | Sim-2 (minimal) |
| A83 | Run robot & weld | M9 + PLC | R2 | Sim-2 |
| A84 | Monitor & handle alarms | M8 + PLC | R2 | Sim-2 (minimal) |
| A85 | Log weld data | M8 (Phase 1), M11 (Phase 2) | R2 | Sim-2 (JSON lines); HW-2 fields |
| A86 | Pause / resume with overlap | M8 + M9 + PLC | Phase 2 | Phase 2 |
| A87 | Export part report | CLI (Phase 1), M11 (Phase 2) | R2 | HW-2 |
| A88 | Manage users & roles | awm_api + UI | R2 | Sim-1 (local login) |
| A91 | Inspect weld | M12 | R2 | HW-2 |
| A92 | Record result / NCR | M12 | R2 | HW-2 |
| A93 | Feed back corrections | M12 → M2–M4 | R3 | Phase 2 (R3) |
| A94 | Seam tracking / vision | M12 plug-in / M9 | Phase 2 | Phase 2 |
| A95 | Cross-section / macro-etch test | M12 (record) + QA | R2 | HW-2 |

## 12. Quality attributes

| Attribute | How the architecture supports it | Verification |
|---|---|---|
| Safety | Hardwired safety chain; Class 4 enclosure and LightWELD interlock loops owned by the PLC; positioner drive STO in the safety chain; reduced-speed dry run before release | REQ-002, safety validation checklist (WBS 6.3) |
| Correctness | Offline 100 % path check, released = executed (D09 as the exact commanded trajectory, B-15), frozen release bundle and context record | M7 report; FAT |
| Accuracy | Frame chain with calibration D08 incl. positioner axis (A55); part location A53; error budget A54 against the weld tolerance | Calibration accuracy check (WBS 4.4 / 4.5); macro-etch A95 |
| Multi-product | Generic pipeline, job package per part | O5: ≥ [TBD] jobs stored and run |
| Traceability | Hashes, program revisions, weld log per seam | Audit of D12 vs D10 |
| Testability | `sim_mock` / `sim_ursim` launches with simulated PLC / LightWELD and mock positioner; unit tests per module; schema validation | CI pipeline on Git |
| Maintainability | One config source, no absolute paths, packages with single responsibility; runtime dependency rule (4.1) | Code review checklist; dependency test in CI (B-06) |
| Security | Roles Operator / Engineer / Admin; all commands through M8 / backend; physical start confirmation (7.3); network and host security (8.1) | Role test cases and port scan in FAT |
| Weld quality | WPS travel speed held in the part frame (A47), speed-scaling rule (6.2), emission-on check | TCP speed of the interpolated D09 trajectory within ±5 % outside the ramps (B-15); macro-etch to the chosen standard (OI-12) |
| Reproducibility | Run record per planning run, deterministic DP tie-break (ADR-19, B-03) | Identical weld / approach / retract payload for identical inputs within one image digest (max abs Δq ≤ 1e-9 rad, same per-point status, header excluded from the hash); air moves: same verdict, cycle time ±10 % (B-11) |
| Performance | One run slot per cell, ψ × branch sweep behind SceneBackend | ≤ 30 s per seam (TST-002 C5, recorded with the backend used); ≤ 5 min per golden-part job (Assumption, to be confirmed after TST-002) |
| Integrity | Frozen release bundle and context record (ADR-20, 5.2), atomic writes, one writer per job | Runtime refuses a bundle with one changed byte (test) |
| Observability | Version stamps and `run_id` in logs (Sim stages); structured logs and `/diagnostics` from HW-1 (12.4) | Every weld log record traceable to software version and release hash |

### 12.1 Safety concept — software view (Proposed, CP-26 as modified by B-18)

The IPC software is **not** a safety-related part of the control system. No safety function depends on it. PLr values come from the risk assessment REQ-002 (OI-31); the table is the starting allocation.

| SF | Safety function | Realised by | Software role |
|---|---|---|---|
| SF1 | Emergency stop of robot, positioner and emission | E-stop chain → UR safety I/O, drive STO, LightWELD interlock loop | None (state shown on HMI) |
| SF2 | Guard door / Class 4 enclosure open → no emission, safe stop | Door switches → safety relay / TwinSAFE → LightWELD interlock loop, UR safeguard stop, drive STO | None |
| SF3 | Emission only with all permissives (door, clamp contact, gas, wire, drive ready) | PLC (LightWELD driver) | Requests only (WeldStart / WeldEnd) |
| SF4 | IPC failure → emission off, axes stopped | PLC heartbeat watchdog: emission off, then positioner stop (6.1) | M8 advances the heartbeat counter; M10 forwards it |
| SF5 | Burn-through: emission while the welding axis is stationary outside the start dwell (all modes; B-17) | PLC (hard-wired Robot Moving and drive in-motion signals, emission-duration cap, 6.1) | None (sends weld_axis, dwell_s, expected_duration_s) |
| SF6 | Robot speed, force and zone limits | UR safety configuration (checksum in the context record), designed by the named safety engineer from the workspace exported by M4 / M7 | Never changes limits (design rule); planning-relevant limits exported as data to `cell.yaml` with the checksum and checked by M4 / M7 with a slow-down margin (HW-1, B-25) |

**SF3–SF5 are process interlocks in the standard PLC until REQ-002 assigns a PLr and a safety-rated realisation (B-18).** The full split into (A) credited safety functions and (B) process interlocks (laser key, passive beam containment with OI-24, fume extraction) is made by the named PLC & safety engineer at the HW-1 gate.

Rules: software may stop the cell but never overrides or emulates SF1–SF6; the `sim` mode of M10 reproduces the DES-002 weld-event table (process interlocks), never the safety functions; every change of SF allocation is a change of REQ-002.

### 12.2 Error budget — first estimate (Proposed, CP-25 as modified by B-21)

Target: the combined error at the seam (root sum of squares of independent sources, plus systematic terms added linearly) must stay inside the allowed seam deviation (OI-16; working value 0.1–0.2 mm, AD5), separately in the lateral direction (in the seam plane: weld position) and along the beam (with nozzle contact: contact force or contact loss, OI-22). Values are Fact or Assumption / [TBD] until measured in A54. The Allocation column is the target for each source; DES-003 targets of A51 / A52 / A53 / A55 are derived from it (each current target alone exceeds AD5).

**Robot weld (robot moves, positioner at rest; robot-only and Mode B)**

| # | Source | Lateral | Along-beam | Allocation | Status | Reduced by |
|---|---|---|---|---|---|---|
| E1 | UR5e pose repeatability | ±0.03 mm | ±0.03 mm | — | Fact (UR data sheet, ISO 9283) | — |
| E2 | UR5e absolute accuracy with nominal kinematics | [TBD] | [TBD] | [TBD] | Fact (bound): nominal kinematics may be off by centimetres — `ur_calibration` README [R14] | Calibrated kinematics (ADR-18), measurement in A54 |
| E3 | TCP calibration residual (A51) | [TBD] | [TBD] | [TBD] | To be measured | Better TCP method, A56 |
| E4 | Fixture frame calibration (A52) | [TBD] | [TBD] | [TBD] | To be measured | Jig reference features |
| E5 | Part tolerance and position in the jig | [TBD] | [TBD] | [TBD] | Customer drawing / jig design | Go / no-go check (A53) |
| E6 | Distortion from manual tack welds (CP-19, OI-26) | [TBD] | [TBD] | [TBD] | Unknown | Tack procedure |
| E7 | Mode B index angle error δ × r | [TBD] | [TBD] | [TBD] | OI-25 | Unidirectional index approach, encoder, A55 |
| E8 | Path discretisation and time parameterisation | ≤ 0.01 mm | ≤ 0.01 mm | — | Assumption | Pitch rule (B-03) |
| E9 | Thermal drift of robot and fixture | [TBD] | [TBD] | [TBD] | Assumption | Warm-up, re-check |
| E11 | Nozzle wear between TCP checks | [TBD] | [TBD] | [TBD] | Assumption | A56 after tip change and at shift start |
| E12 | Shrinkage from earlier seams in the sequence | [TBD] | [TBD] | [TBD] | Unknown | Seam order |
| E13 | Wire aim error (filler wire) | [TBD] | — | [TBD] | Assumption | WPS ψ window, gauge setting at tip change (B-05) |

**Mode C (robot at rest, positioner turns)**

| # | Source | Lateral | Along-beam | Allocation | Status | Reduced by |
|---|---|---|---|---|---|---|
| E1 | UR5e pose repeatability (one static pose) | ±0.03 mm | ±0.03 mm | — | Fact (UR data sheet) | — |
| E2 | UR5e absolute accuracy with nominal kinematics | [TBD] | [TBD] | [TBD] | Fact (bound), [R14] | Calibrated kinematics (ADR-18) |
| E3 | TCP calibration residual (A51) | [TBD] | [TBD] | [TBD] | To be measured | A56 |
| E5 | Part tolerance and position in the holders | [TBD] | [TBD] | [TBD] | Customer drawing / jig design | Go / no-go check (A53) |
| E6 | Distortion from manual tack welds | [TBD] | [TBD] | [TBD] | Unknown | Tack procedure |
| E7 | Positioner angle error δ × seam radius r (0.43° ⇔ 0.1 mm at r = 13.3 mm) | [TBD] | — | [TBD] | OI-25 | Encoder, backlash, A55 |
| E9 | Thermal drift | [TBD] | [TBD] | [TBD] | Assumption | Warm-up |
| E10 | Seam runout about the positioner axis (holder concentricity, part straightness; residual of the Mode C coaxiality tolerance) | [TBD] | [TBD] | [TBD] | Assumption | Holder design, Mode C tolerance (B-03) |
| E11 | Nozzle wear between TCP checks | [TBD] | [TBD] | [TBD] | Assumption | A56 |
| E13 | Wire aim error (filler wire) | [TBD] | — | [TBD] | Assumption | WPS ψ window (B-05) |

Decision rule: the values of OI-30 (with OI-16) are due at the Sim-2 exit. The part-location method (fixed jig only, or a go / no-go touch check) is decided at the HW-1 gate with measured E2. No runtime per-part correction in Phase 1 (OI-43). Fixed reference features on the jig (pin / cone) are ordered as a mechanical item (Nghia Nguyen), needed by HW-1. The budget is reviewed at each calibration (A54).

### 12.3 Test strategy (CP-31 as modified by B-19 / B-24)

| Level | Content | Runs |
|---|---|---|
| Unit | Pure logic without ROS (frame math, mm ↔ m adapter, schemas, DP search on synthetic graphs, context compare) | Every commit (CI) |
| Contract | `awm_interfaces` and OpenAPI: generated clients compile; schema validation of every job file; FK consistency planner vs browser (20 states, 0.01 mm / 0.01°, B-07); runtime dependency rule (B-06) | Every commit |
| Planner regression | Golden part 564082 + analytical test parts (A25) + one 2-solid part: coverage, continuity, clearance, reproducibility (4.3, 12); scene sanity C1 including contact-zone poses (B-04) | Every merge request |
| Integration (Sim-1) | `sim_mock` launch: plan → program → verify → replay on mock hardware | Every merge request / nightly |
| System (Sim-2) | Same simulation release through M8 / M9 / M10 sim on URSim + mock positioner; commanded vs actual against the D09 points (B-15) | Before the Sim-2 exit |
| Fault injection (Sim-2) | DES-002 scenario set against M10 sim (B-18, B-19): start refused below 100 % scaling; scaling change in a weld segment → WeldEnd + FAULT; LaserReady timeout → EmissionOff → FAULT, no retry; EmissionLost mid-seam; Mode C positioner stall; M8 tick stopped while M10 is alive → emission off; door open; LightWELD / gas / wire faults; mixed mode → NOT_READY; one changed byte in D09 → refused | CI on URSim + mock positioner |
| Hardware-in-the-loop | Conformance run of the DES-002 scenario set and PLC interlock matrix against the real PLC program (PLC engineer); Sim-2 fault evidence is provisional until it passes; LightWELD disconnected or key removed; URSim with the designed UR safety configuration | HW-1 |
| FAT | Function and role tests, safety validation checklist (WBS 6.3) | Hardware stage |

Negative tests are required for every rejection rule (unreachable point, collision, contact check, stale context, wrong release scope, speed scaling < 100 %).

### 12.4 Operations and observability (CP-31 as modified by B-18 / B-19)

- Sim stages: software version (Git commit) and image **digest** (not tag) in the context record of D07, D09, D10; `run_id`, `job_id` and `seam_id` in the logs.
- From HW-1: structured logs (JSON lines) from every node and from `awm_api`; health through ROS 2 `/diagnostics` (links to UR, PLC, positioner drive, `move_group`, disk space); backup of the job repository and calibration records to the shared drive daily, restore tested once per release.
- Runbooks (bring-up, manual recovery after FAULT, calibration, dry run) and a support-bundle CLI (logs, rosbag2 excerpt, D09 / D10, context record, versions) are agent deliverables; the agent never touches hardware, the commissioning engineer runs them (B-18, OI-39).

## 13. Open items

| # | Item | Needed for | Owner | Needed by (stage) | Due | Critical path |
|---|---|---|---|---|---|---|
| OI-1 | ~~MIG welder model~~ — moved to Phase 2 (laser only in Phase 1) | — | — | — | — | — |
| OI-2 | IPG LightWELD 1000 automation interface documentation: full 12-pin pinout, trigger / ready / fault signals, preset selection from PLC, Ethernet protocol | DES-002, M3, M10, PLC | Khang Doan / IPG | HW-1 | Sim-2 exit (2026-12-04) | No |
| OI-3 | UR5e PolyScope version, installed URCaps (External Control, Remote TCP) | ADR-01, M9 | [TBD] | Sim-2 | Sim-2 start (2026-11-23) | No |
| OI-4 | Beckhoff PLC model, TwinSAFE vs external safety relay | M10, safety | PLC & safety engineer | HW-1 | Sim-2 exit (2026-12-04) | No |
| OI-5 | Customer Q&A items 1–4 (robot model, sim vs real demo, path source, part positioning); Phase 1 demonstration agreement with the customer: demo part, simulation / hardware, part-location method, STEP only (STL = Phase 2), acceptance evidence (B-22) | ADR-01, A53, 1.2 | Tac Huynh (PM / customer) | Sim-1 | 2026-11-13 | Yes |
| OI-6 | IPC hardware selection | Section 8 | [TBD] | HW-1 | HW-1 entry | No |
| OI-7 | ~~Seam list and path JSON schemas (first version)~~ — closed (B-14): schemas exist in code (v1); v2 frozen 2026-11-04 (B-13) | — | — | — | — | — |
| OI-8 | ~~ADR-09 HMI toolkit~~ — closed (B-14): decided by ADR-09 (Web UI) | — | — | — | — | — |
| OI-9 | ~~Humble → Jazzy migration~~ — closed in Rev 0.3: Jazzy from Phase 1 (ADR-10) | — | — | — | — | — |
| OI-10 | Pipe geometry: 90° or angled intersection, equal or different diameters (equal diameters give singular saddle points) — [R9] Q3 | A25, A44 | PM / customer | [TBD] | [TBD] | No |
| OI-11 | Bevel / groove needed or fillet only; number of passes — [R9] Q4 | A42, A46 | Welding engineer | [TBD] | [TBD] | No |
| OI-12 | Weld quality standard (AWS D1.1, ISO 5817, customer) — [R9] Q5 | A34, A95, FAT | PM / customer | [TBD] | [TBD] | No |
| OI-13 | ~~TCP definition~~ — chosen 2026-10-27: TCP at the nozzle / wire tip, focus offset default 0 (5.3); Assumption until IPG data (OI-2) | A42, A51 | Khang Doan / IPG | — | — | — |
| OI-14 | Positioner drive: driver model, encoder, fieldbus (EtherCAT proposed), payload, angle limits (cables, belts); 1 axis in Phase 1 (from the CAD). Purchase requirement (B-20): closed loop with encoder; cyclic synchronous position / velocity supported; EtherCAT or CANopen CiA 402; hard-wired in-motion and fault outputs; STO. Decide with ADR-22 | ADR-11, M9, `awm_positioner` | Mechanical / electrical | HW-1 (drive purchase) | 2026-11-20 | Yes |
| OI-15 | Real LightWELD head + mount geometry (nozzle, fiber / wire conduit) and clamp geometry for the collision model — TST-002 I2 / I3. Head, mount, jig and holders are now in the cell CAD (head = surface model); nozzle tip / TCP point still to confirm | A45, D14 | Mechanical | HW-1 | HW-1 entry | No |
| OI-16 | Allowed seam deviation (mm) for laser with wobble — [R9] Q7 | A54 | Welding engineer | HW-1 | Sim-2 exit (2026-12-04) | No |
| OI-17 | Redraw Figures 1 (context: LightWELD, Web), 2 (modules: awm_web / awm_api), 3 (data flow: D17–D20), 5 (states: PAUSED, positioner) and 7 (deployment: positioner drive, EtherCAT port); new figure for the positioner modes | Release of docx | Software | [TBD] | [TBD] | No |
| OI-18 | LightWELD wobble range: 0–15 mm (distributor sheet) vs max 5 mm (team note) — confirm in IPG manual | A42, D05 | Khang Doan | [TBD] | [TBD] | No |
| OI-19 | Can the PLC set laser power / wobble directly, or only select presets? | D05, D06, M3 | Khang Doan / IPG | [TBD] | [TBD] | No |
| OI-20 | ~~Positioner needed in Phase 1?~~ — closed 2026-10-27: yes, 1 axis, ROS 2 control (ADR-11) | — | — | — | — | — |
| OI-21 | LightWELD loan window and on-site date vs first laser weld; Class 4 enclosure ready before first emission | Schedule, safety | PM | HW-2 | 2026-11-13 | No |
| OI-22 | Contact at the nozzle tip: rigid tip at 0 mm pre-load, compliant (spring) nozzle or force control; tack beads on the seam. Acceptance: compliance travel ≥ 2 × the along-beam error (12.2) (B-04) | A42, A45, D14 | Welding + robot engineer | HW-1 | HW-1 entry | No |
| OI-23 | Marker components `REF_*` in the Fusion models; remove the duplicate `ASSY, Welding tool`; state the up axis of each export | A18, ADR-16 | Mechanical (Nghia Nguyen) | HW-1 | HW-1 entry | No |
| OI-24 | Upper laser enclosure (Class 4 guard) is not in the cell CAD — needed for collision when the robot reaches high | A18, D14 | Mechanical | HW-2 | HW-2 entry | No |
| OI-25 | Positioner accuracy: belt backlash and stiffness, encoder resolution, against 0.43° (0.1 mm at r 13.3 mm) | A54, A55 | Mechanical | HW-1 | HW-1 entry | No |
| OI-26 | Manual tack welding: where (inside the Class 4 enclosure?), with which tool, and how it is made safe | A81, REQ-002 | Welding engineer / safety | HW-2 | HW-2 entry | No |
| OI-27 | Which holders rotate with the part; positioner angle limits. J2: robot-only first, Mode B only if robot-only coverage < 100 % (decided 2026-10-29). r_max of the rotating set sets the Mode C angular step (B-03) | A49, D14 | Mechanical + software | TST-002 decision | 2026-11-06 | Yes |
| OI-28 | ~~Function owners~~ — closed 2026-10-29: all functions owned by the software agent (Claude Code); direction and simulation tests by Truong Hoang; execution on hardware (calibration, dry run, trial weld, inspection) assigned before the hardware stage. **Exceptions (Proposed, B-18, pending OI-39 / E-03):** the Cell PLC program, DES-002, the alarm catalogue and the UR safety configuration are owned and authored by the named PLC & safety engineer (OI-4); hardware execution by a named commissioning engineer + backup; both named by 2026-11-20. The software agent drafts requirements and runbooks only | — | Linh Tran (amendment) | 2.3 (amendment) | 2026-11-20 (OI-39) | Yes |
| OI-29 | TST-002 decision (Go / Pivot / No-Go, ADR-12) on 2026-11-06, chaired by Linh Tran; spike executed by the software agent 2–5 Nov, checks by Truong Hoang. Missing inputs on 5 Nov → slip ≤ 3 working days, not Pivot. Outcome table in TST-002 Rev 0.3 §6 (B-01) | ADR-12, Sim-1 | Linh Tran, Truong Hoang | Sim-0 exit | 2026-11-06 | Yes |
| OI-30 | Error budget values E2…E13 and the allowed seam deviation (with OI-16), due at the Sim-2 exit; the touch-check decision follows at the HW-1 gate (B-21) | ADR-18, A53, A54 | Welding engineer + software | HW-1 | Sim-2 exit (2026-12-04) | No |
| OI-31 | Risk assessment REQ-002 with PLr for SF1…SF6 (12.1); split into safety functions and process interlocks; confirm the PLC rules of 6.1 (emission follows the welding axis, heartbeat) | 6.1, 12.1, PLC | PLC & safety engineer | HW-1 | Sim-2 exit (2026-12-04) | No |
| OI-32 | Named human reviewers for the safety-relevant packages, the verification-defining files and the PLC program (ADR-23) | ADR-23 | Linh Tran (Engineering Manager) | Sim-2 | 2026-11-20 | Yes |
| OI-33 | ~~Launch names `sim` vs `sim_mock` / `sim_ursim`~~ — closed (B-14): rename the launch in PLAN T0.4 as Rev 0.5 decided | — | — | — | — | — |
| OI-34 | ~~Code migration plan Rev 0.2 → Rev 0.5 / 0.6~~ — closed (B-08): migration plan = PLAN.md §2 (keep / replace per package); T0.1 limited to kept code | — | — | — | — | — |
| OI-35 | ~~Sim-1 scope~~ — closed (B-12, B-14): moved to PLAN.md (ordered slice, principle 9) | — | — | — | — | — |
| OI-36 | Requalification after a calibration-only change (envelope, re-validated release); Phase 1 rule meanwhile: automatic full re-plan and re-verify of all jobs (B-25) | ADR-17, A51, A74 | Truong Hoang + welding engineer | HW-2 | HW-2 gate | No |
| OI-37 | **Escalation E-01 (board 2026-10-30) — not decided.** Is the TST-002 (ADR-12) decision held on 2026-11-06, chaired by Linh Tran, with the frozen inputs of B-01…B-05, and missing inputs treated as a slip of at most 3 working days rather than a Pivot? Options: A yes, as proposed; B decide on 2026-11-06 with whatever evidence exists, missing evidence = Pivot signal; C no fixed date. Board recommendation: A | ADR-12, TST-002, OI-29 | Linh Tran (Engineering Manager), with Truong Hoang | Sim-0 exit | 2026-11-06 | Yes |
| OI-38 | **Escalation E-02 — not decided.** Amend ADR-17 (decided 2026-10-27) with a `trial` release scope so that A72 / A73 run through M8 / M9 / M10, and make production a promotion of the same frozen bundle? Options: A amend now (scope table and CLI refusal before Sim-2, gate code at HW-2); B decide at the HW-1 gate, scope check built as a data table either way; C keep ADR-17 and run A72 / A73 outside M8 / M9 (breaks AD8). Board recommendation: A, with B as fallback; C not acceptable | ADR-17, A72, A73, A74 | Truong Hoang | Sim-2 (scope table) | 2026-11-20 (or HW-1 gate, option B) | No |
| OI-39 | **Escalation E-03 — not decided.** Amend OI-28 so that a named human owns and authors the Cell PLC program, DES-002, the alarm catalogue and the UR safety configuration, plus a named commissioning engineer and a Laser Safety Officer? Options: A amend (PLC & safety engineer and commissioning engineer + backup by 2026-11-20, LSO before HW-2); B amend only for the PLC program; C keep OI-28. Board recommendation: A | OI-28, ADR-23, 2.3 | Linh Tran, with Truong Hoang | Sim-2 | 2026-11-20 | Yes |
| OI-40 | **Escalation E-04 — not decided.** Phase 1 deliverable on 2026-12-04? Options: A a simulation-proven system (Sim-2), HW-1 / HW-2 dated in PM-004; B move the Phase 1 end after HW-2; C compress the hardware stages into December (not recommended). Board recommendation: A, aligned with the customer agreement (OI-5) | 1.2, 2.3, PM-004 | Tac Huynh (project lead) and Linh Tran | Sim-1 | 2026-11-13 | Yes |
| OI-41 | **Escalation E-05 — not decided.** Do the J1 / J2 WPS use filler wire, and from which side (ψ window of the TST-002 decision)? Options: A autogenous, free ψ 0–350°; B filler wire leading, ψ window ±15° (Assumption); C unknown by 2026-11-04: report both cases and decide ADR-12 on the wire-window case. Board recommendation: answer by 2026-11-04, otherwise C | ADR-12, D05, TST-002 | Truong Hoang (with the welding engineer and Khang Doan) | TST-002 decision | 2026-11-04 | Yes |
| OI-42 | **Escalation E-06 — not decided.** Rebuild the IPC runtime and CI image from the public `ros:jazzy` base (non-root, pinned digest, micromamba OCC lock) before HW-1, although the environment is fixed to the demo-based image? Options: A yes, as a HW-1 entry item built in parallel, dev container switched only after parity tests; B use the demo-based image on the IPC; C switch now, during Sim-1. Board recommendation: A | 8, 8.1, ADR-10 | Truong Hoang | HW-1 | Sim-2 exit (2026-12-04) | No |
| OI-43 | **Escalation E-07 — not decided.** If the error budget shows the fixed jig cannot meet the band, may a per-part measured correction re-solve IK at runtime as an exception to design rule 1? Options: A no exception (reject out-of-tolerance parts or widen the band, OI-16); B bounded exception (same ψ and configuration label, deterministic re-solve, all checks re-run, bound in `cell.yaml`, logged); C offline re-plan per part before welding. Board recommendation: decide at the HW-1 gate with measured E2, OI-16, OI-30; default A; B only with written bounds and ADR-23 review | ADR-18, 12.2 | Truong Hoang | HW-1 | HW-1 gate | No |
| OI-44 | **Escalation E-08 — not decided.** Which part is demonstrated to the customer in Phase 1 (564082 or 152056 from the customer Q&A, wiki L-009), and is part detection by sensor expected? Options: A 564082 only, fixed jig, STEP only; B add 152056 as a second job; C part-detection sensor in Phase 1 (own CP and stage). Board recommendation: A, plus B if its STEP is available by 2026-11-13; C only as a staged CP after HW-1 | OI-5, 1.2, A53 | Tac Huynh | Sim-1 | 2026-11-13 | Yes |

**Dated dependencies outside the OI list (B-22).** `moveit_py` / distance API check: spike day 1. DES-002 weld-event behaviour table (draft by the software agent, B-18): 2026-11-20. Rotary ground contact for Mode C and fixed reference features on the jig: mechanical design inputs now (Nghia Nguyen). DES-002 complete and REQ-002 Rev 0.2: Sim-2 exit. UR safety configuration design and the 'Robot Moving' threshold measured at the slowest travel speed: HW-1 entry. Laser Safety Officer: HW-2 entry. PLAN.md gets §6 'Risks and triggers' and PROGRESS.md a weekly dependency check.

## Appendix A. Cross-reference to the UR5e Welding Robot Project Tracker [R9]

Review date 2026-10-22. Status: **Adopted** = added to this document and to DES-003 Rev 0.2; **Slot** = kept as Phase 2 extension point; **Not adopted** = conflicts with a decision, reason given; **PM** = belongs to WBS / schedule, not to the architecture.

| Tracker item | Status | Where |
|---|---|---|
| Pipeline step 0 / 5.8 Calibration (TCP 4–6 orientations, fixture 3-point, error budget) | Adopted | A51–A54, M5 |
| Pipeline step 0 / 5.8 Positioner axis calibration | Adopted (Phase 1) | A55, ADR-11 |
| Step 1 / 5.1 Load CAD, units in m, heal, split solids, glTF | Adopted | A11–A13, M1 |
| 5.1 / 5.2 STL fallback (RANSAC) | Slot | A16 |
| Step 2 Place part in virtual cell, reachability | Adopted | A15, A17, D19 |
| Step 3 / 5.2 Automatic seam detection (section, concave edges, merge, classify) | Not adopted — removed from scope 2026-10-22 (ADR-03) | — |
| 5.2 Analytical T / L pipe seam generator | Adopted | A25, M2 |
| Step 4 / 5.3 Seam editor (reverse, start point, split, order) | Adopted | A22, UI-W |
| Step 5 / 5.9 WPS library with versioning | Adopted | A34, M3 |
| Step 6 / 5.4 Seam frame, torch angles, stick-out, weave patterns | Adopted (weave replaced by LightWELD wobble) | A41, A42 |
| 5.4 Multi-pass offsets | Slot | A46 |
| Step 7 / 5.5 Positioner angle, 7-axis coordination, Options A / B | Adopted for Phase 1 (Modes B / C); continuous A1 / A2 = Slot | A49, ADR-11 |
| Step 8 / 5.6 IK with redundant tool yaw, graph search, singularities | Adopted | A44, 4.3, ADR-12, TST-002 |
| 5.6 Wrist-3 soft limit / cable wrap | Adopted | A48 |
| 5.6 Collision model (torch neck + cable, ACM wire tip) | Adopted | A45, D14 |
| Step 9 / 5.6 Freespace planning, approach / retract 30–50 mm | Adopted | A43 |
| 5.5 Time parameterisation in the part frame | Adopted | A47 |
| 5.5 Speed-scaling trap | Adopted (robot only in Phase 1) | 6.2, M9, A83 |
| Step 10 Simulation with failing points in red; dry run checklist | Adopted | A71, A72, UI-W |
| Step 11 / 5.7 Weld sequence, watchdog | Adopted in PLC / M10, adapted to laser (pre-flow, emission-on check, ramp-down, wire stop, post-flow) | A63, 6.2, ADR-04 |
| 5.7 ROS2 `weld_controller` as single welder owner via UR I/O | Not adopted — PLC owns welder / laser (ADR-04) | ADR-04 |
| 5.9 Pause / resume with overlap | Slot | A86, PAUSED state |
| Step 12 / 5.9 rosbag2 + per-part report | Adopted | A85, A87, D17, D18 |
| 5.9 Data Part → Job → Run, calibration version flag | Already covered | 5.2 (hashes) |
| Architecture: Web UI (React + Three.js) + FastAPI backend, roles | Adopted (ADR-09 decided) | 4.5, 7.3, 7.4 |
| Architecture: `weld_interfaces` shared package | Already covered | `awm_interfaces` |
| Design principle: simulate first | Adopted | ADR-13, principle 7 |
| Design principle: thin slice first | Adopted | principle 9 |
| Deployment: PREEMPT_RT kernel, shared Docker image, CI | Adopted | Section 8 |
| W9.3 / Q5 Macro-etch against quality standard | Adopted | A95, OI-12 |
| Assumptions A1–A6, Open Questions Q1–Q7 | Mapped | OI-1, OI-10…OI-16 |
| Risks R1–R9 | PM — to be merged into APLSMN601-REQ-002 / PM-005 Risk Register | — |
| Roadmap P0–P3 (~5–6 months), Tasks W0–W9, Gantt, team split | PM — to be reconciled with WBS PM-003 and schedule PM-004 (charter: R1–R5, Phase 1 by early Dec 2026) | — |
| Scope: MIG only, T / L pipe joints | Not adopted — Phase 1 is laser only (LightWELD), multi-product | 1.2 |
