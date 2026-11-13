# APLSMN601 Automated Welding Machine — Software Architecture Description

| Field | Value |
|---|---|
| Doc. No. | APLSMN601-DES-001 |
| Rev | 0.5 |
| Date | 2026-10-29 |
| Status | Accepted by Truong Hoang on 2026-10-29 for team use (Engineering Manager approval pending). Prepared by the software agent from the decisions of 2026-10-27 (CP-9…CP-23) and the document review of 2026-10-29 |
| Prepared by | Truong Hoang |
| Reviewed by | [TBD] |
| Approved by | Linh Tran (Engineering Manager) |
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

## 1. Introduction

### 1.1 Purpose

This document defines the software architecture of the APLSMN601 Automated Welding Machine: the software modules, their responsibilities, the data that flows between them, the interfaces to the robot, cell PLC and IPG LightWELD 1000 laser welder, and the runtime and deployment structure. Each function A1–A9 of the Function Register (APLSMN601-DES-003) is allocated to exactly one owning module.

### 1.2 Scope

Phase 1 proof of concept: UR5e on the Pismo frame, **laser welding with an IPG LightWELD 1000** (CW 1000 W, wobble head, wire feeder), **1-axis rotary positioner controlled by ROS 2** (decision 2026-10-27), STEP import, cell and fixture geometry from the mechanical CAD (Fusion), manual seam selection in the 3D simulator view, offline path generation, simulation, calibration, PLC-controlled laser, Web UI. Phase 2 items (MIG process, STL fallback A16, multi-pass A46, continuous 7-axis motion with the positioner, pause / resume A86, seam tracking A94) are shown only as extension points. Automatic seam detection is not part of the product scope (decision 2026-10-22).

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
| [R8] | ISO 10218-1/-2, ISO 12100, ANSI Z136.1, ISO/IEC/IEEE 12207 |
| [R9] | UR5e_Welding_Robot_Project_Tracker (Google Sheet, based on the project roadmap document of 15 Oct 2026): Pipeline, Architecture, Module Details 5.1–5.9, Tasks W0–W9, Assumptions A1–A6, Open Questions Q1–Q7, Risks R1–R9 |
| [R10] | APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.2 (results in Rev 0.3) |
| [R11] | IPG LightWELD 1000 W specification (distributor data sheet) and LightWELD XR user manual (Section 5.6 customer interface connections, 12-pin connector) — checked 2026-10-22; IPG automation interface documentation still to be obtained (OI-2) |
| [R12] | Cell CAD `APLSMN601_Automated Weldment System.step` (Fusion export, 2026-10-27): enclosure lower frame, Pismo plate, UR5e, LightWELD head and mount, 1-axis positioner, jig and holders, golden part `564082-Assembly-3 parts` |
| [R13] | Change proposal DES-001 Rev 0.4 → Rev 0.5 (`docs/generated/DES-001_change_proposal.md` in the software repository), CP-9…CP-23 |

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
| D01…D22 | Data items of the Function Register Rev 0.5 (D01…D21); D22 added by CP-64 (not in DES-003 Rev 0.5) |
| ψ (tool yaw) | Rotation of the weld head about its beam axis — redundant DOF used for reach, collision and singularity avoidance |
| ACM | Allowed Collision Matrix (MoveIt) |
| CTWD | Contact-tip-to-work distance (stick-out) |
| Positioner | 1-axis rotary positioner (7th, external axis) carrying fixture and part — Phase 1, controlled by ROS 2 (ADR-11) |
| Mode B / Mode C | Positioner motion modes of Phase 1 (6.3): B = rotate, stop, weld; C = robot at rest, positioner turns during emission (seam coaxial with the positioner axis) |
| Marker component | Small named component in a CAD assembly whose origin and axes define a frame (`REF_*`, ADR-16) |
| Release scope | `simulation` or `production` (ADR-17) |
| ADS | Beckhoff Automation Device Specification protocol (TCP 48898) |

## 2. Architecture drivers

### 2.1 Requirements that shape the architecture

| # | Driver | Source | Architectural consequence |
|---|---|---|---|
| AD1 | Weld paths from CAD without teaching (O1) | Charter | Exact B-Rep geometry (OCCT), not mesh; offline planning chain A1→A4→A6 |
| AD2 | Many products in one cell (O5) | Charter | No part-specific code. All part data lives in a job package; one generic pipeline |
| AD3 | Laser welding with IPG LightWELD 1000 in Phase 1; MIG later (O2, O4; decision 2026-10-22) | Charter, decision | Process abstraction kept: one weld-event model; Phase 1 has only the LightWELD driver in the PLC. Weaving is done by head wobble, not by the robot |
| AD4 | Robot ↔ LightWELD ↔ PLC communication (O3) | Charter, decision | PLC is the single owner of LightWELD trigger / enable / preset signals and of the laser interlocks |
| AD5 | Accuracy ±0.1–0.2 mm (laser; wobble may widen the allowed band — OI-16) | Charter risk 02 | Explicit frame chain (part → fixture → base → TCP) with calibration data D08 |
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
7. **Simulate first.** Code runs against simulated drivers from day one; switching to hardware is a launch parameter, not a code change (ADR-13).
8. **mm everywhere in project data** (decision 2026-10-22, ADR-14). All project-owned data, JSON schemas, `awm_interfaces` messages, UI and reports use mm, degrees and seconds. Third-party ROS packages (ur_description, MoveIt2, TF, ur_robot_driver) are fixed to metres and radians by REP 103; the single mm ↔ m adapter in M0 is the only place that converts.
9. **Thin slice first.** Build one end-to-end slice (one part, one seam, one WPS) through the whole chain before widening features.

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
| Customer / remote viewer | Views jobs, simulation and reports in a browser (read-only role) |
| Git server | Source code (Rule 4 of the project README) |
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

![Figure 2 – Module architecture](APLSMN601-DES-001_figures/fig2_modules.png)

### 4.2 Module catalogue

| Module | ROS2 package / project | Responsibility | Functions owned | Phase |
|---|---|---|---|---|
| M0 Core | `awm_core` | Data classes, JSON schemas (D03–D13, D16–D19, D22), frame transforms, job repository API, mm ↔ m adapter for third-party ROS packages (ADR-14) | — (shared) | 1 |
| M1 CAD Import | `awm_cad` | Load STEP AP203/214/242 with OCCT, split solids, normalise units, check and heal, tessellate (glTF display + collision mesh), create part record with file hash, export the 3D view model D20 (glb per face / edge + topology), place part in the virtual cell (D19, source `study` or `fixture_cad`); import the cell / fixture CAD assemblies (D21) into the cell model D14 and the nominal calibration (A18); STL fallback with RANSAC cylinder fit. Runs as a separate process (ADR-15) | A11, A12, A13 (render), A14, A15, A17 (with M4), A18 · A16 (Phase 2) | 1 / 2 |
| M2 Seam Definition | `awm_seam` | Resolve picked edge IDs to exact B-Rep curves, chain tangent-continuous edges into one seam, seam attributes (joint type, closed / open, side, start point, direction, split, sequence, leg size; tack welds are made by hand, A81), seam list export with face normals; analytical T / L pipe seam generator as test tool | A21, A22, A24, A25 (test) | 1 |
| M3 Process Parameters | `awm_process` | WPS library maintenance with versioning (released WPS read-only), laser mode per seam (CW / HPP pulse), LightWELD preset and parameter assignment | A31, A32, A33, A34 | 1 |
| M4 Path Planner | `awm_path` | Seam discretisation and seam frame, head orientation and focus offset (wobble is a head parameter, no robot weave), approach / retract and air moves, IK with tool-yaw and IK-branch graph search (ADR-12), collision check, time parameterisation at WPS travel speed in the part frame, wrist-3 soft limit / cable-wrap check, positioner mode and angles (Modes C / B); multi-pass in Phase 2 | A41–A45, A47–A49 · A46 (Phase 2) | 1 / 2 |
| M5 Calibration | `awm_calib` | TCP calibration (4–6 orientations, least squares), fixture frame (3-point), part location (fixed-jig check, touch sensing later), error budget, positioner axis, calibration records | A51–A55 | 1 |
| M6 Program Generator | `awm_program` | Transform path to base frame, build executable program (joint trajectory segments + weld events), store with revision | A61, A62, A63, A64 | 1 |
| M7 Verification & Release | `awm_verify` | Run program in URSim, playback in the Web simulation view, reach / collision / cycle-time report, dry-run and trial-weld records, release workflow (scopes `simulation` / `production`, ADR-17) and lock | A71, A72, A73, A74 | 1 |
| M8 Cell Sequencer | `awm_cell` | Cycle state machine, job selection, start / stop / reset, alarm handling and recovery, speed-scaling supervision; pause / resume with overlap in Phase 2 | A81, A82 (supervisor), A84 · A86 (Phase 2) | 1 / 2 |
| M9 Motion Executor | `awm_executor` | Replay released trajectory segments on the UR5e through ur_robot_driver and on the positioner through its joint trajectory controller, start-state check, speed override for air moves only, speed-scaling monitor, feedback | A83 (motion) | 1 |
| M10 PLC Bridge | `awm_plc_bridge` | ADS link IPC ↔ PLC: commands, weld parameters, status, alarms, heartbeat; `sim` mode emulates PLC + LightWELD with the same interface (ADR-13); LightWELD only — the positioner is not behind the PLC (ADR-11) | A82 (IPC side), A63 (runtime events) | 1 |
| M11 Weld Logger | `awm_logger` | Structured weld log per seam (part, program rev, set vs actual parameters, alarms, times, operator), rosbag2 run recording, part report export | A85, A87 | 1 |
| M12 Inspection & Feedback | `awm_inspect` | Inspection record (visual and macro-etch), NCR link, correction request back to A2–A4 (new job revision) | A91, A92, A93, A95 · A94 (Phase 2) | 1 / 2 |
| UI Web front-end | `awm_web` (React + TypeScript, react-three-fiber, urdf-loader) | One browser app with role-based screens: job library, 3D seam picker, simulation, WPS / preset library, production HMI, calibration, reports, users (Section 4.5) | A13, A17, A21, A22 (UI), A71 (display), A74, A81, A84 (display), A87, A88 (UI) | 1 |
| UI backend | `awm_api` (FastAPI + rclpy node) | REST + WebSocket API (7.4), authentication and roles (A88), command checks, job repository access, calls to M1–M12 | A88 | 1 |
| Cell model | `awm_description` | URDF / xacro: UR5e (`ur_description`) + LightWELD head and mount (incl. fiber / wire-feed conduit section near the head for collision) + Pismo frame + positioner joint with its rotating holders + fixture + clamps, SRDF, MoveIt config; static geometry, head and positioner links generated from the Fusion CAD by A18 (ADR-16) | D14 | 1 |
| Bring-up | `awm_bringup` | Launch files: `sim_mock` (Sim-1, mock hardware), `sim_ursim` (Sim-2, URSim + mock positioner), `real` (UR5e + positioner drive), `workbench`, `web` (awm_api + static front-end); RViz only for developer debugging | — | 1 |
| Interfaces | `awm_interfaces` | ROS2 msg / srv / action definitions (Section 7.1) — the contract between modules, reviewed by the team before coding | — | 1 |
| Positioner driver | `awm_positioner` | ros2_control hardware interface for the rotary table (7th joint) in the same controller manager as the UR driver; `mock_components` in simulation; drive on EtherCAT proposed (OI-14) | A83 (positioner motion), A55 (runtime) | 1 |
| Cell PLC program | TwinCAT project `APLSMN601_CellPLC` (separate Git repo) | Cycle permissives, robot handshake, LightWELD driver (trigger, preset, interlocks, wire feeder), positioner drive ready / fault monitoring (emission off on fault), safety supervision, alarms | A81, A82, A84 (PLC side), A83 (process) | 1 |

### 4.3 Engineering chain (offline)

M1 → M2 → M3 → M4 → M6 → M7 runs in the Workbench. Each step writes its output into the job package and records the hash of its inputs, so a later step can detect stale inputs (for example, a seam list that was edited after the path was planned).

M4 uses the MoveIt2 `move_group` node with the cell model D14 and the collision scene D19 for IK, Cartesian path and collision services. The Workbench starts its own planning-only ROS2 graph (no robot hardware); this reuses the MoveIt configuration of the Tac demo. IK is enumerated with an analytic UR solver (all 8 branches, ADR-12); pick_ik returns one solution and is not used for the sweep. Planning uses the active calibration D08; a calibration change makes D07 stale (plan after calibration).

M4 process-motion algorithm (ADR-12, validated by TST-002):

1. Discretise the seam by arc length (1–3 mm); seam frame X = tangent, Z = bisector of the face normals pointing out of the material (up, towards the head; for a butt joint between tubes n1 = n2 and Z is the surface normal), Y = Z × X; the beam axis is −Z (A41).
2. Apply work / travel angle and focus offset (wobble is set in the LightWELD preset, not by the robot) and the 360° closing overlap (A42).
3. Per point, sample tool yaw ψ (e.g. 10° steps) × the 8 UR5e IK branches; drop states in collision, beyond joint limits or near a wrist / shoulder / elbow singularity (A44, A45). With the positioner (A49): seams coaxial with its axis use Mode C (one robot pose, the positioner turns; the head is checked against the rotating part and holders over the full turn); other seams use Mode B (a positioner angle per arc segment, searched together with ψ and the IK branch; robot-only first).
4. Layered graph (layer = point, node = valid joint state, edges only within one IK branch with |Δq| ≤ 5° per step), edge cost = joint change + clearance and singularity penalties + wrist-3 travel; shortest path gives a continuous chain without configuration flips.
5. Smooth, time-parameterise at the WPS travel speed in the part frame (A47), check wrist-3: ±270° absolute soft limit and ±180° travel within one seam (A48). Mode C: positioner speed = travel speed / seam radius.
6. Approach / retract 30–50 mm along the beam axis; OMPL air moves between seams from the chosen joint states and positioner index moves with the robot parked; every segment checked densely (≤ 0.02 rad joint step) (A43).

A seam that fails any step is rejected with the failing points listed (shown red in the simulation view); the engineer then changes direction, start point, head angles or part placement (A15, A17, A22, A42).

### 4.4 Runtime chain (online)

M8 Cell Sequencer is the single coordinator. It loads a released program (D10; scope `simulation` only in simulation mode, ADR-17), checks calibration (D08 hash in the program must match the active calibration), then runs the seam loop (Section 6.2) by calling M9 for motion (robot and positioner) and M10 for weld events. M11 records everything. The Operator HMI talks only to M8.

### 4.5 User interface design (ADR-09: Web)

**Structure.** Browser (`awm_web`) ⇄ HTTPS REST + WebSocket ⇄ `awm_api` (FastAPI + rclpy) ⇄ ROS2 services / actions of M1–M12. The browser never calls ROS directly. The physical Start button is wired to the PLC.

**3D view model (D20).** M1 exports for every part revision:

- `view.glb` — one mesh node per B-Rep face (`face_id`) and one polyline node per B-Rep edge (`edge_id`), in mm, edges tessellated with chord ≤ 0.05 mm;
- `topology.json` — edge → adjacent faces, edge length, closed flag, tangent continuity to neighbouring edges, geometric fingerprint (used to re-match seams when the STEP revision changes).

The browser picks only IDs; `awm_api` sends the IDs to M2, which resolves them to the exact B-Rep curve. Display geometry is never used as a weld path.

**Screens and roles**

| # | Screen | Main actions | Roles |
|---|---|---|---|
| S1 | Job library | Create job from STEP, open, copy, revision list, status (draft / simulated / released) | Engineer, Admin; Operator read |
| S2 | Seam picker (3D simulator) | Click edge → auto-chain tangent edges → seam; select / deselect, reverse direction, start point, split, order, joint type; cell, fixture, positioner and robot shown in the same scene | Engineer |
| S3 | Process | Assign WPS / LightWELD preset per seam, focus offset, travel speed, wobble values (display of preset) | Engineer |
| S4 | Simulation | Plan request, trajectory playback with scrubber, points coloured by status (OK / collision / out of reach / near singularity / joint limit), robot configuration (shoulder / elbow / wrist −1 / 0 / +1) and tool-yaw override per seam, positioner mode (B / C) and angles, placement study (candidates with coverage, A17), cycle time | Engineer |
| S5 | Release | Review report D15, dry-run and trial-weld records, approve → lock | Engineer request, Admin approve |
| S6 | Production HMI | Select released job, Start request, Stop, Reset, seam progress, live LightWELD status, alarms EN / VN | Operator |
| S7 | Calibration | Guided TCP, fixture and positioner-axis calibration, error budget, history | Admin |
| S8 | Reports & logs | Weld log, part report export (D18) | All (read) |
| S9 | Users | Users, roles, permissions | Admin |
| S10 | Cell model | Import the cell / fixture CAD (D21), marker and CAD-map check report | Admin |
| S11 | Manual control (Jog) | Joint jog of the 6 UR5e joints and the positioner joint; Cartesian jog (frame toggle Tool / Base) acting at `tcp_laser`; move-to a typed pose; teach the cell home (D22); live readout of both `tcp_laser` and `tool0` (mm / deg). Simulation only: model and analytic UR IK / FK in `awm_api`, no controller, no hardware, no collision check (CP-64) | Admin |

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

Data items D01–D21 are those of the Function Register Rev 0.5; D22 was added by CP-64 (not in DES-003 Rev 0.5). All lengths in mm (ADR-14). The as-built merge of D04 and D07 (DES-004) is reversed: seams and paths are separate files.

| ID | Data item | Producer | Consumers | Format / schema | Storage |
|---|---|---|---|---|---|
| D01 | STEP file | Customer | M1 | .step AP203/214/242 | job/`model.step` (copy, read-only) |
| D02 | B-Rep model | M1 | M1, M2, M4 (collision mesh) | OCCT TopoDS_Shape in memory; .brep cache | job/`cache/` |
| D03 | Part record | M1 | M6, M7, M11 | `part.schema.json` | job/`part.json` |
| D04 | Seam list | M2 | M3, M4 | `seams.schema.json` (curve, face normals n1 / n2, closed / open, attributes, fingerprint) | job/`seams.json` |
| D05 | WPS library | Welding engineer | M3, M4, M7 | `wps.schema.json` (one file per WPS; references the LightWELD preset number and its parameters: power, mode, wobble, wire feed, gas) | cell/`wps/` |
| D06 | Seam process parameters | M3 | M4, M6 | `process.schema.json` | job/`process.json` |
| D07 | Weld path (part frame): tool poses in mm, timed joint path incl. positioner angle and mode, per-point status, D08 hash | M4 | M6, M7 | `path.schema.json` | job/`path.json` |
| D08 | Calibration data (TCP, fixture frame, part offset, positioner axis; `NOMINAL` from the cell CAD until measured) | M5 (nominal: M1 / A18) | M4, M6, M8, M9 | `calibration.schema.json` | cell/`calibration/<date>_<id>.json` + `active.json` link |
| D09 | Robot program | M6 | M7, M9 | `program.schema.json`: JSON, ROS 2 joint-trajectory segments (robot 6 joints + positioner) + weld events; URScript export = fallback | job/`programs/<rev>/program.json` |
| D10 | Released program | M7 | M8, M9 | `release.schema.json` (program rev, hashes, scope `simulation` / `production`, approver, date) | job/`release.json` |
| D11 | I/O & handshake signal list | DES-002 (design) | M6, M10, PLC | `io_map.yaml` generated from the ICD sheet | cell/`io_map.yaml` |
| D12 | Weld log | M11 | M12 | SQLite table + CSV export | `logs/weld_log.sqlite` |
| D13 | Inspection record | M12 | M12 (A93) | `inspection.schema.json` | job/`inspections/` |
| D14 | Cell model | M1 (A18) from D21 + `ur_description` | M4, M7 | URDF / xacro / SRDF | `awm_description` package |
| D15 | Simulation / verification report | M7 | M7 (A74) | JSON + PDF / screenshots | job/`programs/<rev>/verify/` |
| D16 | Cell & runtime configuration | Design | all runtime modules, M4 (soft limits) | YAML (ROS2 parameters) | `awm_bringup/config/` |
| D17 | Run recording | M11 | M12 | rosbag2 (joints, I/O, status) | `logs/bags/<date>_<job>/` |
| D18 | Part weld report | M11 | M12, customer | PDF / CSV | job/`reports/` |
| D19 | Part placement & collision scene | M1 (A15, A17) | M4, M7 | `placement.schema.json` (incl. `source` = `study` / `fixture_cad`) + meshes | job/`placement.json`, job/`cache/` |
| D20 | 3D view model | M1 (A13) | UI (S2, S4), M2 | `view.glb` + `topology.json` | job/`cache/view/` |
| D21 | Cell & fixture CAD | Mechanical design (Fusion) | M1 (A15, A18) | STEP AP214 / AP242 assembly, mm, marker components `REF_*` | cell/`cad/` (cell), job/`fixture.step` (fixture) |
| D22 | Cell home (taught) | `awm_api` (S11, `PUT /cell/home`; CP-64) | M4 (IK seed, follow-up), M8 (go home, follow-up) | `home.schema.json`: robot 6 joints + positioner angle, deg; `source` = `taught`; stale when the `frames.yaml` hash in `inputs` is not the current one. Supersedes the nominal SRDF home where read; absent = SRDF nominal | cell/`model/home.json` |

![Figure 3 – Data flow between modules](APLSMN601-DES-001_figures/fig3_dataflow.png)

### 5.2 Job package

```
<repo_root>/
├── cell/
│   ├── wps/                      D05  WPS-<id>.json
│   ├── calibration/              D08  <date>_<id>.json, active.json
│   ├── cad/                      D21  cell STEP assembly + cad_map.yaml (component → role, up axis)
│   ├── model/                    D14  meshes and frames generated by A18
│   │   └── home.json             D22  taught cell home (S11, CP-64)
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
│       │   └── verify/           D15
│       ├── release.json          D10
│       ├── inspections/          D13
│       └── reports/              D18
└── logs/
    ├── weld_log.sqlite           D12
    └── bags/                     D17  rosbag2 per run
```

Rules:

- Every JSON file has `schema_version`, `created_by`, `created_at` and `inputs` (list of input file names with SHA-256). A module refuses inputs whose hash does not match.
- Seam IDs (`S01`, `S02` …) are assigned by the user and kept stable. Each seam stores a geometric fingerprint of its B-Rep edges so it can be re-matched when the STEP revision changes.
- A released program is read-only. Any change (seam, parameter, path, calibration) creates a new program revision that must be verified and released again (A93 → A2–A4).
- The repository root is a parameter. Phase 1: a local folder on the IPC, backed up to the shared drive. Phase 2: database / MES if needed.
- A `production` release requires `placement.json` with `source: fixture_cad` (ADR-17).

### 5.3 Coordinate frames

![Figure 4 – Frame chain](APLSMN601-DES-001_figures/fig4_frames.png)

| Frame | Defined by | Stored in |
|---|---|---|
| `world` | Cell origin on the Pismo frame | D14 |
| `base` | UR5e base | D14 (nominal), D08 (measured if needed) |
| `fixture` | Fixture reference points (3-point method, A52) | D08 |
| `part` | STEP model origin; nominal pose on fixture from fixture design | D14 nominal, D08 offset from A53 |
| `tcp_laser` | On the beam axis at the nozzle / wire tip, i.e. the contact point on the seam; tool Z = beam direction; `focus_offset_mm` (D06, default 0) moves the planned point along the beam axis. Assumption until IPG data (OI-2); `tcp_mig` reserved for Phase 2 | D08 (also written to the UR installation) |
| `positioner_axis`, `positioner_flange` | Rotary table axis (A55) and the flange at angle θ; `fixture` and `part` are children of `positioner_flange` | D08 (nominal from D21) |

**Axis convention (decision 2026-10-27).** Z is up in `world` and `base`. Seam frame: X = travel direction, Z = bisector of the face normals pointing out of the material, Y = Z × X; the beam axis is −Z. CAD files may use another up axis (the Fusion cell CAD is Y-up); A18 applies a fixed rotation per file, stated in the CAD map.

**Nominal frames from CAD (ADR-16).** `world`, `base`, `positioner_axis`, `fixture`, `part`, `tool0` and `tcp_laser` are read from marker components `REF_WORLD`, `REF_UR_BASE` (aligned with the UR controller `base` frame, which is `base_link` rotated 180° about Z in `ur_description`), `REF_POSITIONER`, `REF_FIXTURE`, `REF_PART`, `REF_TOOL0`, `REF_TCP`.

D07 is always in the `part` frame. M6 computes `base_T_tcp = base_T_axis · Rz(θ) · flange_T_fixture · fixture_T_part · part_T_pose · tcp⁻¹` with the active D08 (θ = positioner angle of the point or segment) and writes the D08 hash into D09. The runtime refuses a program whose calibration hash is not the active one (fixes F14).

### 5.4 Collision model — data sources

Collision checking (FCL in MoveIt2) works on triangle meshes and primitives, never on B-Rep. STEP stays the source of truth; meshes are generated from it with controlled tolerance.

| Object | Source file | Collision representation | Notes |
|---|---|---|---|
| Part | Customer STEP (D01) | Mesh from M1, chord ≤ 0.1 mm near seams, coarser elsewhere; each triangle keeps its `face_id` | ACM: only the `nozzle_tip` link (last 5 mm of the nozzle, TCP at its end) may touch the part; part, fixture and holders rotate with the positioner |
| Fixture, clamps | Fusion CAD (D21) | Simplified STL / convex hulls in D14 / D19 | 3 mm padding |
| LightWELD head + mount, fiber and wire-feed conduit near the head | Head and mount from the Fusion cell CAD (D21; the head is a surface model) | Convex hulls; nozzle split into `nozzle_tip` / `nozzle_body`; conduit as capsules | Fiber bend radius ≥ 50 mm checked by A48 |
| UR5e | ur_description | Vendor collision meshes | — |
| Pismo frame, table | Fusion CAD (D21) | Boxes / convex hulls | — |
| Positioner (rotating holders, shaft side) | Fusion CAD (D21) | Convex hulls moving with `positioner_joint` | 3 mm padding; Mode C checked over the full turn |

No additional file type is needed from the customer.

**Clearance values (D16).** `padding_mm` = 3 (every link except `nozzle_tip`), `weld_clearance_mm` = 5 (head body, mount and conduit during weld segments; TST-002 C2), `air_clearance_mm` = 10 (approach, retract, air and index moves; TST-002 C4). Every segment is checked densely (≤ 0.02 rad joint step), not only at trajectory waypoints.

## 6. Interaction and state viewpoint

### 6.1 Cell state machine (M8 + PLC)

![Figure 5 – Cell state machine](APLSMN601-DES-001_figures/fig5_states.png)

| State | Entry condition | Main action | Exit |
|---|---|---|---|
| INIT | Power-up | Start ROS2 nodes, connect UR, PLC and positioner drive, reference (home) the positioner, load D16 | All links OK → NOT_READY |
| NOT_READY | Link loss, safety not OK, robot not in remote | Show reason on HMI | All permissives OK → IDLE |
| IDLE | Permissives OK | Operator selects released job | Job valid (release + calibration hash) → READY |
| READY | Job loaded, robot and positioner at home | Operator loads and clamps the parts and tacks them by hand (OI-26); wait for cycle start (outside guard, door closed) | Start → LOCATE_PART |
| LOCATE_PART | Start accepted | A53: fixed-jig check or touch sensing (later) | Part found → WELDING; not found → FAULT |
| WELDING | Part located | Seam loop (6.2) | Last seam done → CYCLE_END; pause request → PAUSED (Phase 2); any error → FAULT |
| PAUSED (Phase 2) | Operator pause | Stop in place, emission off, retract (A86) | Resume → back up a few mm along the seam, restart weld sequence → WELDING; stop → FAULT |
| CYCLE_END | All seams done | Go home, close weld log, PLC end-of-cycle | Part unloaded → READY |
| FAULT | Any alarm | Emission off, wire feed off, controlled stop of robot and positioner, alarm text | Operator reset + cause cleared → IDLE |
| E_STOP | Safety chain open (hardware) | Robot and power stopped by hardware | Safety reset → NOT_READY |

The PLC keeps its own mirror of the cycle state and can force FAULT at any time (LightWELD fault, positioner drive fault, interlock open, door open, heartbeat loss from IPC). Heartbeat loss acts as the watchdog: the PLC switches emission off without waiting for the IPC.

### 6.2 Seam execution (weld events)

![Figure 6 – One seam: sequence](APLSMN601-DES-001_figures/fig6_sequence.png)

Design rule: **process events happen only while the robot is at rest at segment boundaries** (approach end, weld end). This removes the need for hard real-time coupling between ROS2 trajectory execution and laser switching in Phase 1, and works for both MIG and laser:

1. M9 moves to the approach pose, then the weld start pose (stored trajectory segments).
2. M8 → M10 → PLC: `WeldStart(seam_id, wps_id, preset)`. PLC checks interlocks (door, E-stop, external interlock loops, workpiece-clamp contact), selects the LightWELD preset, gas pre-flow, wire-feed ready, then trigger → emission.
3. PLC → M10 → M8: `LaserReady` / emission-on feedback within the timeout from D06; on timeout one automatic retry, then FAULT. Dwell time from D06.
4. M9 executes the weld segment at constant TCP speed (time-parameterised by M4 from the WPS travel speed; fixes F12).
5. At weld end: `WeldEnd` → power ramp-down / dwell (if the preset supports it), emission off, wire-feed stop / retract, post-flow; PLC confirms `EmissionOff`. Closed 360° seams end with the overlap planned in A42.
6. M9 retracts and moves to the next seam (collision-checked air move).

**Mode C (positioner weld, seam coaxial with the positioner axis).** Steps 1–3 as above with the robot at the weld start pose and the positioner at rest; step 4 is a positioner weld segment executed by M9 (robot at rest, positioner turns at ω = travel speed / seam radius plus the closing overlap); step 5 starts when the positioner is at rest again. **Mode B:** the positioner index move to the planned angle is made with emission off, before step 1. Rule: WeldStart and WeldEnd only while robot **and** positioner are at rest.

Speed-scaling rule: `scaled_joint_trajectory_controller` slows the robot when the UR speed slider or safety scaling is below 100 %, which changes the travel speed and the heat input. M9 monitors the speed-scaling value; weld segments start only at 100 % and any change during a weld segment → `WeldEnd` + FAULT. Air moves may run with override. The same rule applies to a Mode C positioner weld segment: it starts only at 100 % (no override on weld segments). In Phase 1 the robot and the positioner never move together during emission, so no common scaling is needed; continuous 7-axis motion (Phase 2) must apply one scaling to both or stop (ADR-11).

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

**Phase 1 decision (2026-10-27).** The mechanical design contains a 1-axis positioner (motor, coupling, shaft, two timing-belt stages, holders on the jig; horizontal axis). It is part of Phase 1 (OI-20 closed). On the golden part 564082, joint J1 (pipe nipple ↔ solid) is coaxial with the axis → Mode C; J2 (solid ↔ elbow) is 76.6 mm off the axis with its axis perpendicular to it → robot alone or Mode B. Accuracy: an angular error δ gives r·δ at the seam; at r = 13.3 mm, 0.1 mm ⇔ 0.43°, so backlash and encoder resolution must be checked (OI-25).

## 7. Interface viewpoint

### 7.1 Software interfaces (ROS2)

| Interface | Type | Provider → user | Content |
|---|---|---|---|
| `/awm/cell/command` | service | M8 ← HMI | select_job, start (request only — physical start button required, 7.3), stop, reset, home; pause / resume in Phase 2 |
| `/awm/cell/state` | topic (latched) | M8 → HMI, M11 | state, job, seam, progress, active alarms |
| `/awm/execute_segment` | action | M9 ← M8 | program rev, segment id (robot, positioner, weld or index segment), speed override (air / index moves only); feedback %, result |
| `/awm/weld_event` | action | M10 ← M8 | WeldStart / WeldEnd with seam, WPS, LightWELD preset; result LaserReady / EmissionOff / fault |
| `/awm/plc/status` | topic | M10 → M8, M11, HMI | PLC state, permissives, LightWELD status (ready, emission, fault, interlocks), wire feeder, gas, active preset |
| `/awm/alarm` | topic | all → M8, M11, HMI | alarm id, source, severity, text EN / VN |
| MoveIt2 `compute_ik`, `compute_cartesian_path`, `check_state_validity` | services | move_group → M4 | planning (Workbench graph only) |
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
| IPC ↔ positioner drive | EtherCAT (CiA 402) through `ethercat_driver_ros2`, own IPC port (proposal, OI-14) | position set-points, actual position, drive status | Yes (ros2_control cycle, PREEMPT_RT) |
| Positioner drive ↔ PLC / safety | Hard-wired | drive ready / fault to the PLC; STO from the safety chain | Safety function, independent of software |
| Safety chain | Hardwired safety relay / TwinSAFE ↔ UR safety I/O | E-stop, guard door, laser interlock, robot safeguard stop | Safety function, independent of software |

Ownership rule: the PLC is the only device that switches the LightWELD. The IPC requests; the PLC decides based on interlocks. The positioner is moved only by ROS 2 (M9); its safe stop is hardware (STO).

### 7.3 User roles and command security

| Role | Allowed |
|---|---|
| Operator | Select and run released jobs; load, clamp and tack parts; start / stop / reset; view status and alarms |
| Engineer | Create and edit jobs, pick and edit seams (only this role, decision 2026-10-27), placement study, run simulation and dry run, request release |
| Admin | Calibration, cell configuration D16, cell model import (A18), manual jog and teach home on S11 (cell-setup screen, CP-64), user management, approve release |

Rules:

1. Every motion or process command (start, jog, home, weld) goes through `awm_api` and M8, which check role, cell state and permissives. A UI never talks to the robot or PLC directly (no rosbridge for commands). S11 (CP-64) is simulation-only and issues no motion command; this rule applies to any controller-backed jog (OI-29).
2. A Start from any UI is a request; motion starts only after the physical start button at the cell (outside the guard). E-stop is always hardware.
3. Status to the UI is pushed at 10–30 Hz over WebSocket — display only, not used for control.

### 7.4 Web API (`awm_api`, first version)

| Endpoint | Method | Purpose | Role |
|---|---|---|---|
| `/auth/login` | POST | Login, token with role | All |
| `/jobs` · `/jobs/{id}` | GET / POST | Job list, create from STEP upload, job detail and revisions | Engineer |
| `/jobs/{id}/view` | GET | D20 view model (`view.glb`, `topology.json`) | All |
| `/jobs/{id}/seams` | GET / PUT | Seam list D04 (picked edge IDs + attributes, mm) | Engineer |
| `/jobs/{id}/process` | GET / PUT | Process assignment D06 | Engineer |
| `/jobs/{id}/placement` | GET / PUT | D19 placement: candidates (`study`) or import from the fixture CAD (`fixture_cad`) | Engineer |
| `/jobs/{id}/plan` | POST | Request planning (M4) — async, progress over WebSocket | Engineer |
| `/jobs/{id}/simulation` | GET | Trajectory samples + per-point status for playback | Engineer |
| `/jobs/{id}/release` | POST | Release request / approval (M7), scope `simulation` or `production` | Engineer / Admin |
| `/wps` | GET / POST | WPS / LightWELD preset library D05 | Welding engineer |
| `/cell/command` | POST | select_job, start request, stop, reset, home (→ M8) | Operator |
| `/calibration` | GET / POST | Calibration runs and records D08 | Admin |
| `/cell/model` | GET / POST | Cell CAD import (D21 → D14), marker check report | Admin |
| `/cell/fk` | POST | Joints (deg) → `tool0` and `tcp_laser` poses (mm, quaternion); numeric, no controller (CP-64) | Operator / Engineer / Admin (CELL_READ) |
| `/cell/ik` | POST | Target `tcp_laser` pose + current joints → nearest IK solution (deg); problems `ik-no-tcp` 409, `ik-unreachable` / `ik-singular` 422 (CP-64) | Admin (CELL_WRITE) |
| `/cell/home` | GET / PUT | Read D22 with staleness (404 `no-home`, 422 `home-invalid`) / teach D22 from the current joints (409 `no-frames`, 422 validation) (CP-64) | GET CELL_READ; PUT Admin (CELL_WRITE) |
| `/reports/{part}` | GET | Part report D18 | All |
| `/users` | GET / POST | Users and roles | Admin |
| `ws://…/status` | WebSocket | Cell state, seam progress, LightWELD status, alarms (10–30 Hz) | All |

## 8. Deployment viewpoint

| Node | Hardware (proposal) | Software |
|---|---|---|
| IPC | Industrial PC, 8+ cores, 32 GB RAM, NVIDIA GPU optional, 3 × Ethernet (cell LAN, office, EtherCAT to the positioner drive) | Ubuntu 24.04 with PREEMPT_RT (real-time) kernel as recommended for ur_robot_driver, ROS 2 Jazzy (ADR-10), MoveIt2, ur_robot_driver, `awm_api` (FastAPI) + static `awm_web` build behind a reverse proxy, M1 CAD process with pythonocc-core in its own conda environment (ADR-15); Docker image for reproducible build |
| Operator panel / engineering PC | Panel PC or tablet at the cell; any PC on the cell LAN | Chromium / Chrome in kiosk mode for S6; normal browser for engineering screens |
| UR5e controller | e-Series | PolyScope 5.x [TBD], External Control URCap, safety configuration, TCP from D08 |
| Cell PLC | Beckhoff CX / IPC with TwinCAT 3 [TBD] | `APLSMN601_CellPLC`, EtherCAT I/O, TwinSAFE or external safety relay |
| Development | Engineer laptop, WSL2 (host Ubuntu 22.04 left unchanged) | ROS 2 Jazzy inside the Docker image of the Tac demo (same image as the IPC); URSim container for simulation (`sim_ursim` launch; `sim_mock` for Sim-1); CI builds all packages and runs unit tests on every merge request |

Network (proposal, to be fixed in DES-002): isolated cell LAN 192.168.10.0/24 — IPC .10, UR5e .20, PLC .30, laptop .100. The IPC second port connects to the office network for Git and the shared drive only; the third port is the EtherCAT segment of the positioner drive.

## 9. Architecture decisions

| ADR | Decision | Status | Alternatives / reason |
|---|---|---|---|
| ADR-01 | **Motion execution through ROS2**: M6 stores pre-planned, time-parameterised joint trajectory segments; M9 replays them with ur_robot_driver. URScript export (Remote TCP / Toolpath URCap) kept as fallback (WBS 4.3). | Proposed — decide at G2 | URScript `movel` / `movep` gives native constant TCP speed and runs if the IPC stops, but duplicates planning logic and is harder to simulate with MoveIt. ROS2 reuses the Tac demo and the simulation chain. |
| ADR-02 | **B-Rep CAD kernel OCCT (pythonocc-core)**; STL only for display / collision meshes | Proposed | Mesh pipeline of the demo loses exact edges and needs hard-coded seams (F03, F04) |
| ADR-03 | **Manual seam selection only**: seams are picked edge by edge in the 3D simulator view, once per part number, then simulated and released. Automatic seam detection is removed from scope | Confirmed 2026-10-22 (replaces "automatic detection in Phase 2") | Scope control (charter risk 04); the people who know the weld symbols choose the seams |
| ADR-04 | **PLC owns the IPG LightWELD 1000 and the laser interlocks** (trigger, preset, interlock loops, wire feeder); IPC ↔ PLC over ADS; process events only at segment boundaries. Weld sequence and watchdog of [R9] 5.7 are implemented in the PLC / M10 | Confirmed for Rev 0.3 (2026-10-22) | [R9] proposes a ROS2 `weld_controller` node driving the welder over UR I/O or fieldbus. Rejected for Phase 1: bypasses PLC interlocks and the Class 4 laser interlock chain; the timing need it addresses is removed by events at rest |
| ADR-05 | **Plan offline, replay online** — no weld-path planning at runtime; runtime checks only start state and hashes | Proposed | Demo plans at runtime and accepts 35 % paths (F01) |
| ADR-06 | **One robot (UR5e) in the Phase 1 cell model** — remove GP25, rail and gripper from D14 and sequencer; the part sits on the fixture of the 1-axis positioner (ADR-11) | Confirmed for Rev 0.3 (2026-10-22); updated 2026-10-27 | Fixed Pismo frame in Phase 1 (F10); keep the demo dual-robot code in a Phase 2 branch |
| ADR-07 | **File-based job repository + SQLite weld log** | Proposed | Simple, versionable, enough for Phase 1; database / MES in Phase 2 |
| ADR-08 | **Python for engineering modules, C++ for real-time-relevant ROS2 nodes (M9)** | Proposed | Python is fastest for OCCT / geometry work; executor reuses welder_action_server C++ code |
| ADR-09 | **Web UI for Workbench and Operator HMI**: React + TypeScript + react-three-fiber + urdf-loader front-end (`awm_web`), FastAPI + rclpy backend (`awm_api`) on the IPC; design in Section 4.5, API in 7.4 | Decided 2026-10-22 | Seam picking in 3D by engineers and operators, headless Linux IPC, access from panel PC / tablet / customer. Rejected: Qt + OCCT (desktop only, robot simulation needs RViz separately); TwinCAT HMI (far from job and 3D data). Risk: 3D front-end effort — mitigated by picking IDs only (D20) and reusing urdf-loader |
| ADR-10 | **ROS 2 distribution: Ubuntu 24.04 + ROS 2 Jazzy from Phase 1**, developed inside the Docker image of the Tac demo; host WSL Ubuntu 22.04 stays unchanged | Changed in Rev 0.3 (2026-10-22) — replaces the Rev 0.2 Humble decision | The demo Docker image already runs Jazzy and TST-002 uses it, so no Humble environment is needed. Humble EOL May 2027 vs Jazzy EOL May 2029 (REP 2000); no Humble → Jazzy migration later (OI-9 closed) |
| ADR-11 | **1-axis rotary positioner in Phase 1, controlled by ROS 2**: `awm_positioner` ros2_control hardware component in the same controller manager as the UR driver, own joint trajectory controller; Phase 1 motion Modes C (robot at rest, positioner turns during emission, coaxial seams) and B (rotate – stop – weld); continuous 7-axis motion (Options A1 / A2) in Phase 2; safety in hardware (STO, drive fault to the PLC). Analysis in Section 6.3 | Decided 2026-10-27 (replaces the Phase 2 slot of 2026-10-22) | The mechanical design already contains the positioner; 360° seams coaxial with its axis need no robot orbit. PLC-driven positioner rejected: not deterministic and it blocks Option A1. Hardware open (OI-14, OI-25) |
| ADR-12 | **Process-motion planner: MoveIt2 + custom tool-yaw / IK-branch graph search** (Section 4.3), as tested in TST-002. Tesseract + tesseract_planning (Descartes, TrajOpt, OMPL) is the fallback if TST-002 ends in Pivot / No-Go | Proposed — decide after TST-002 (29 Oct 2026) | pick_ik (Tac demo) returns one IK solution and is not used for the sweep; analytic UR IK gives all 8 branches; Tesseract is widely used in ROS-Industrial welding apps but adds a second planning stack [R9 5.6] |
| ADR-13 | **Simulated drivers behind the same interfaces**: URSim / UR fake hardware for the robot, `sim` mode of M10 for PLC + LightWELD, `mock_components` positioner joint (Phase 1) | Proposed | Hardware arrives late (LightWELD on loan, automation interface open, OI-2); lets the full runtime chain run in CI and simulation from day one [R9] |
| ADR-14 | **mm for all project data**: schemas, `awm_interfaces` messages (fields named `*_mm`), UI, reports. One adapter in M0 converts to metres where project code calls third-party ROS packages | Decided 2026-10-22 | Engineers and drawings work in mm. Third-party packages (URDF, TF, MoveIt2, ur_robot_driver) stay in metres per REP 103 and cannot be rescaled safely; risk of unit mistakes is contained by the single adapter and unit tests on it |
| ADR-15 | **CAD module M1 runs as a separate process** with pythonocc-core ≥ 7.9 (conda-forge; tested with 8.0.1 / Python 3.10), exchanging files (STEP in, JSON / glb / mesh out) with `awm_api` and M2 | Proposed | pythonocc-core is distributed through conda; keeping it out of the ROS 2 Python environment avoids package conflicts. Same split suits the Web architecture |
| ADR-16 | **Cell, fixture, head and positioner geometry from Fusion STEP assemblies**: read by M1 with OCCT XCAF (component tree, names, placements); frames from marker components `REF_*`; component → role map kept as data (`cell/cad/cad_map.yaml`); fixed CAD → `world` axis rotation per file; the UR5e model always from `ur_description`; no CAD → URDF plug-in | Decided 2026-10-27 | The mechanical team designs in Fusion; one source for nominal geometry avoids hand-typed transforms. Fusion construction geometry and joint origins are not exported to STEP, hence marker components. Read test on the first cell CAD: 98 MB in 56 s, names and placements kept |
| ADR-17 | **Release scopes**: `simulation` (after a PASS simulation report only; never valid for production) and `production` (PASS report, dry run, trial weld, error budget, placement source `fixture_cad`, Admin approval). The runtime accepts only `simulation` in simulation mode and only `production` with real hardware | Decided 2026-10-27 | Sim-2 exercises the same release and hash checks as production without faking A72 / A73 records |

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

| Function | Name | Module | Release |
|---|---|---|---|
| A11 | Load STEP file | M1 | R1 |
| A12 | Check & heal model | M1 | R1 |
| A13 | Display 3D model | M1 + UI-W | R1 |
| A14 | Create part record | M1 | R1 (basic), R5 (library) |
| A15 | Place part in virtual cell | M1 + UI-W | R1 |
| A17 | Study part placement | M1 + M4 + UI-W | R1 (manual candidates), R3 (automatic) |
| A18 | Import cell CAD | M1 | R1 |
| A16 | Load STL (fallback) | M1 | Phase 2 |
| A21 | Select seam edges | M2 + UI-W | R1 |
| A22 | Define seam attributes | M2 | R1 |
| A24 | Export seam list | M2 | R1 |
| A25 | Generate analytical pipe seam | M2 | R1 (test ground truth) |
| A31 | Select laser mode per seam | M3 | R2 |
| A32 | Look up WPS | M3 | R2 |
| A33 | Assign weld parameters | M3 | R2 |
| A34 | Maintain WPS library | M3 | R2 |
| A41 | Discretize seam | M4 | R1 |
| A42 | Set head orientation, focus & wobble | M4 | R1 (angles), R2 (wobble from preset) |
| A43 | Add approach / retract moves | M4 | R1 |
| A44 | Solve IK & check reach | M4 | R1, R3 (automatic) |
| A45 | Check collisions | M4 | R1, R3 (automatic) |
| A46 | Generate multi-pass offsets | M4 | Phase 2 |
| A47 | Time-parameterize at travel speed | M4 | R1 (nominal), R2 (verified on weld) |
| A48 | Check joint soft limits & cable wrap | M4 | R1 |
| A49 | Coordinate positioner angle | M4 | R1 (Modes C / B) |
| A51 | Calibrate TCP | M5 | R2 |
| A52 | Calibrate fixture frame | M5 | R2 |
| A53 | Locate part | M5 | R2 (fixed jig), R4 (touch sensing) |
| A54 | Check error budget | M5 | R2, R4 |
| A55 | Calibrate positioner axis | M5 | R2 |
| A61 | Transform path to robot frame | M6 | R1 (nominal), R2 (calibrated) |
| A62 | Generate motion program | M6 | R1, R3 (URScript fallback) |
| A63 | Insert weld & I/O events | M6 + M10 | R2 |
| A64 | Store & transfer program | M6 | R1 |
| A71 | Simulate program | M7 | R1 |
| A72 | Dry run on robot | M7 + M9 | R1 / R2 |
| A73 | Trial weld on coupon | M7 | R2 |
| A74 | Release program | M7 | R2 |
| A81 | Load part & start cycle | M8 + UI-H + PLC | R2 |
| A82 | PLC – robot – LightWELD handshake | M8 + M10 + PLC | R2 |
| A83 | Run robot & weld | M9 + PLC | R2 |
| A84 | Monitor & handle alarms | M8 + PLC | R2 |
| A85 | Log weld data | M11 | R2 |
| A86 | Pause / resume with overlap | M8 + M9 + PLC | Phase 2 |
| A87 | Export part report | M11 | R2 |
| A88 | Manage users & roles | awm_api + UI | R2 |
| A91 | Inspect weld | M12 | R2 |
| A92 | Record result / NCR | M12 | R2 |
| A93 | Feed back corrections | M12 → M2–M4 | R3 |
| A94 | Seam tracking / vision | M12 plug-in / M9 | Phase 2 |
| A95 | Cross-section / macro-etch test | M12 (record) + QA | R2 |

## 12. Quality attributes

| Attribute | How the architecture supports it | Verification |
|---|---|---|
| Safety | Hardwired safety chain; Class 4 enclosure and LightWELD interlock loops owned by the PLC; positioner drive STO in the safety chain; reduced-speed dry run before release | REQ-002, safety validation checklist (WBS 6.3) |
| Correctness | Offline 100 % path check, released = executed, hash checks | M7 report; FAT |
| Accuracy | Frame chain with calibration D08 incl. positioner axis (A55); part location A53; error budget A54 against the weld tolerance | Calibration accuracy check (WBS 4.4 / 4.5); macro-etch A95 |
| Multi-product | Generic pipeline, job package per part | O5: ≥ [TBD] jobs stored and run |
| Traceability | Hashes, program revisions, weld log per seam | Audit of D12 vs D10 |
| Testability | `sim_mock` / `sim_ursim` launches with simulated PLC / LightWELD and mock positioner; unit tests per module; schema validation | CI pipeline on Git |
| Maintainability | One config source, no absolute paths, packages with single responsibility | Code review checklist |
| Security | Roles Operator / Engineer / Admin; all commands through M8 / backend; physical start confirmation (7.3) | Role test cases in FAT |
| Weld quality | WPS travel speed held in the part frame (A47), speed-scaling rule (6.2), emission-on check | Speed error < 5 %; macro-etch to the chosen standard (OI-12) |

## 13. Open items

| # | Item | Needed for | Owner |
|---|---|---|---|
| OI-1 | ~~MIG welder model~~ — moved to Phase 2 (laser only in Phase 1) | — | — |
| OI-2 | IPG LightWELD 1000 automation interface documentation: full 12-pin pinout, trigger / ready / fault signals, preset selection from PLC, Ethernet protocol | DES-002, M3, M10, PLC | Khang Doan / IPG |
| OI-3 | UR5e PolyScope version, installed URCaps (External Control, Remote TCP) | ADR-01, M9 | [TBD] |
| OI-4 | Beckhoff PLC model, TwinSAFE vs external safety relay | M10, safety | PLC & safety engineer |
| OI-5 | Customer Q&A items 1–4 (robot model, sim vs real demo, path source, part positioning) | ADR-01, A53 | PM / customer |
| OI-6 | IPC hardware selection | Section 8 | [TBD] |
| OI-7 | Seam list and path JSON schemas (first version) | R1 | Software |
| OI-8 | ADR-09 HMI toolkit | R2 | Software + PLC |
| OI-9 | ~~Humble → Jazzy migration~~ — closed in Rev 0.3: Jazzy from Phase 1 (ADR-10) | — | — |
| OI-10 | Pipe geometry: 90° or angled intersection, equal or different diameters (equal diameters give singular saddle points) — [R9] Q3 | A25, A44 | PM / customer |
| OI-11 | Bevel / groove needed or fillet only; number of passes — [R9] Q4 | A42, A46 | Welding engineer |
| OI-12 | Weld quality standard (AWS D1.1, ISO 5817, customer) — [R9] Q5 | A34, A95, FAT | PM / customer |
| OI-13 | ~~TCP definition~~ — chosen 2026-10-27: TCP at the nozzle / wire tip, focus offset default 0 (5.3); Assumption until IPG data (OI-2) | A42, A51 | Khang Doan / IPG |
| OI-14 | Positioner drive: driver model, encoder, fieldbus (EtherCAT proposed), payload, angle limits (cables, belts); 1 axis in Phase 1 (from the CAD) | ADR-11, M9, `awm_positioner` | Mechanical / electrical |
| OI-15 | Real LightWELD head + mount geometry (nozzle, fiber / wire conduit) and clamp geometry for the collision model — TST-002 I2 / I3. Head, mount, jig and holders are now in the cell CAD (head = surface model); nozzle tip / TCP point still to confirm | A45, D14 | Mechanical |
| OI-16 | Allowed seam deviation (mm) for laser with wobble — [R9] Q7 | A54 | Welding engineer |
| OI-17 | Redraw Figures 1 (context: LightWELD, Web), 2 (modules: awm_web / awm_api), 3 (data flow: D17–D20), 5 (states: PAUSED, positioner) and 7 (deployment: positioner drive, EtherCAT port); new figure for the positioner modes | Release of docx | Software |
| OI-18 | LightWELD wobble range: 0–15 mm (distributor sheet) vs max 5 mm (team note) — confirm in IPG manual | A42, D05 | Khang Doan |
| OI-19 | Can the PLC set laser power / wobble directly, or only select presets? | D05, D06, M3 | Khang Doan / IPG |
| OI-20 | ~~Positioner needed in Phase 1?~~ — closed 2026-10-27: yes, 1 axis, ROS 2 control (ADR-11) | — | — |
| OI-21 | LightWELD loan window and on-site date vs first laser weld; Class 4 enclosure ready before first emission | Schedule, safety | PM |
| OI-22 | Contact at the nozzle tip: rigid tip at 0 mm pre-load, compliant (spring) nozzle or force control; tack beads on the seam | A42, A45, D14 | Welding + robot engineer |
| OI-23 | Marker components `REF_*` in the Fusion models; remove the duplicate `ASSY, Welding tool`; state the up axis of each export | A18, ADR-16 | Mechanical (Nghia Nguyen) |
| OI-24 | Upper laser enclosure (Class 4 guard) is not in the cell CAD — needed for collision when the robot reaches high | A18, D14 | Mechanical |
| OI-25 | Positioner accuracy: belt backlash and stiffness, encoder resolution, against 0.43° (0.1 mm at r 13.3 mm) | A54, A55 | Mechanical |
| OI-26 | Manual tack welding: where (inside the Class 4 enclosure?), with which tool, and how it is made safe | A81, REQ-002 | Welding engineer / safety |
| OI-27 | Which holders rotate with the part; positioner angle limits. J2: robot-only first, Mode B only if robot-only coverage < 100 % (decided 2026-10-29) | A49, D14 | Mechanical + software |
| OI-28 | ~~Function owners~~ — closed 2026-10-29: all functions owned by the software agent (Claude Code); direction and simulation tests by Truong Hoang; execution on hardware (calibration, dry run, trial weld, inspection) assigned before the hardware stage | — | — |
| OI-29 | Controller-backed / real-robot jog from S11: enabling device (deadman), reduced speed, Remote mode, hardwired E-stop, REQ-002 hazard analysis. S11 is simulation only (CP-64); this must be designed before S11 may drive any controller (mock, URSim or real) | S11, ADR-13, REQ-002 | Software + safety engineer |
| OI-30 | Wire D22 into the planner IK seed (`ChainParams.home_joints_rad`) and the runtime "go home" (Sim-2 state machine, 6.1); until then the SRDF nominal home stays in use. Also: default step sizes and positioner soft-limit source (URDF limits) | M4, M8, D22 | Software |

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
