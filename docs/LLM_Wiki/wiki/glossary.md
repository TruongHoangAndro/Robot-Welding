---
id: glossary
title: Glossary
type: overview
domains: [PM, WELD, COLL, UR5E]
tags: []
status: current
updated: 2026-10-27
---

# Glossary

Sources are cited by document; source pages are created during ingest.

| Term | Meaning | Source |
|---|---|---|
| ACM | Allowed Collision Matrix (MoveIt) — pairs of links / objects allowed to touch | [F] DES-001 §1.4 |
| ADS | Beckhoff Automation Device Specification protocol (TCP 48898); IPC ↔ PLC link | [F] DES-001 §1.4, §7.2 |
| B-Rep | Boundary representation — exact CAD geometry (faces, edges) from STEP via OCCT | [F] DES-001 ADR-02 |
| Cell PLC | Beckhoff TwinCAT 3 controller: cycle, interlocks, LightWELD I/O, safety | [F] DES-001 §1.4 |
| CTWD | Contact-tip-to-work distance (stick-out); MIG term, Phase 2 | [F] DES-001 §1.4 |
| FCL | Flexible Collision Library — collision checking in MoveIt2 on meshes / primitives | [F] DES-001 §5.4 |
| Golden part | Reference workpiece used to prove the chain (TST-002 / Sim-1: 564082 Elbow) | [F] Charter §09; TST-002 §2 |
| IK branch | One of the 8 analytic inverse-kinematics solutions of the UR5e for one TCP pose (shoulder / elbow / wrist ±) | [F] TST-002 §1; DES-001 §4.3 |
| IPC | Industrial PC running Ubuntu 24.04 + ROS 2 Jazzy (planning and cell software) | [F] DES-001 §1.4 |
| Job | One part number + revision with its seams, parameters, paths and programs | [F] DES-001 §1.4 |
| Job package | Folder per job holding D01–D20 files with schema_version and input hashes | [F] DES-001 §5.2 |
| Ladder graph (DP) | Layered graph: layer = seam point, node = valid joint state (ψ × IK branch); dynamic programming finds one continuous path | [F] DES-001 §4.3; Collision Solution slide 5 |
| LightWELD | IPG LightWELD 1000 fiber-laser welder: base unit + weld head (1.3 kg) + 10 m fiber + wire feeder; presets | [F] DES-001 §1.4 |
| Mock hardware | MoveIt2 / ros2_control fake hardware used in Sim-1 | [F] PM-004 v2 §0 |
| OCCT | Open CASCADE Technology (pythonocc-core) — CAD kernel, separate process (ADR-15) | [F] DES-001 ADR-02, ADR-15 |
| OMPL | Open Motion Planning Library — air moves between seams | [F] DES-001 §4.3 |
| Part-contact circuit | LightWELD fires only when the nozzle touches the part and closes the clamp circuit | [F] LASER WELDING slides 6–7 |
| Positioner | Rotary table as 7th (external) axis — Phase 2 slot (ADR-11) | [F] DES-001 §1.4 |
| Runtime | Software that runs released jobs on the cell (online) | [F] DES-001 §1.4 |
| Sim-0 / Sim-1 / Sim-2 | Setup + spike / MoveIt mock playback / same trajectories on URSim | [F] PM-004 v2 §0 |
| Speed scaling | UR controller slows trajectory progress (slider or safety); weld segments start only at 100 % | [F] DES-001 §6.2 |
| TCP | Tool centre point; for LightWELD: focus point / wire tip (OI-13 open) | [F] DES-001 §5.3 |
| URSim | Universal Robots offline simulator (Sim-2) | [F] PM-004 v2 §0 |
| Wobble | Beam oscillation made by the weld head (amplitude, frequency) — replaces robot weaving | [F] DES-001 §1.4 |
| Workbench | Engineering application used to create and release jobs (offline) | [F] DES-001 §1.4 |
| WPS | Welding procedure specification; here linked to LightWELD presets (D05) | [F] DES-001 §5.1 |
| ψ (tool yaw) | Rotation of the weld head about its beam axis — redundant DOF for reach, collision and singularity avoidance | [F] DES-001 §1.4 |
