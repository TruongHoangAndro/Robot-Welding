# Research brief — collision-free weld path planning for a UR5e laser welding cell

Date: 2026-10-27 · Author: Truong Hoang · Context: APLSMN601 (Apillis), Sim stage
Use: paste this brief into a Claude Code session opened in this folder (ARS plugin is installed at project scope),
e.g. `/ars-lit-review` followed by the text of sections 1–4, or `/ars-3w` for a paper shortlist.

## 1. Problem
A UR5e (6 DOF) carries an IPG LightWELD 1000 laser head. Seams are picked on a STEP model (B-Rep edges) and the weld
path is planned offline, then replayed. The welding task fixes 5 DOF of the tool (position + beam axis); the rotation
ψ about the beam axis is free. The planner must find one continuous, collision-free, singularity-free joint path for the
whole seam (no IK-branch change), or reject the seam. Parts: pipe elbows / nipples / flanges with closed 360° seams.

Current proposal (DES-001 ADR-12, tested in TST-002 spike 23–29 Oct 2026):
discretise seam (1–3 mm) → seam frame (X tangent, Z bisector of face normals) → sample ψ (10° steps) × 8 analytic UR
IK branches → drop states in collision / joint limits / near singularity / low clearance (MoveIt2 + FCL) →
ladder-graph dynamic programming (cost = joint motion + deviation + low-clearance penalty + wrist-3 travel) →
approach / retract along beam axis + OMPL air moves. Fallback: Tesseract (Descartes, TrajOpt, OMPL).

## 2. Research questions
- RQ1 Is "sampling the free tool axis × IK branches + graph search" the established method for redundant 5-DOF
  welding tasks? How does it compare with Descartes-style semi-constrained planning, TrajOpt / optimisation, and
  sampling-based (RRT / PRM) task-constrained planning, on completeness, path quality, and planning time?
- RQ2 When no valid state exists at some seam points (TST-002 criterion C2 fails), what do published systems do:
  relax work / travel angles, tilted tool mount, part re-orientation / placement optimisation, seam splitting,
  positioner?
- RQ3 Can closed 360° pipe seams be welded by a 6-DOF arm without a positioner? What reach / wrist-wrap limits are
  reported (TST-002 C8, DES-001 OI-20)?
- RQ4 (fact-check) Library assumptions: analytic UR IK returns 8 solutions; pick_ik returns one; opw_kinematics does
  not fit the UR wrist; Descartes / Tesseract status on ROS 2 Jazzy.

## 3. Scope
- Include: robotic arc / laser welding path planning, redundancy resolution about the tool axis, collision-aware
  Cartesian / process path planning, offline programming from CAD, industrial 6-DOF arms (UR, KUKA, ABB, Fanuc).
- Exclude: seam tracking / vision (Phase 2), multi-pass offset generation, weld-process physics, mobile robots.
- Period: 2010–2026, plus foundational works cited by them.
- Source types: peer-reviewed papers, theses; software documentation (ROS-Industrial, MoveIt, Tesseract) read
  directly — not via citation indexes.

## 4. Keywords
"redundancy resolution" welding robot tool axis; "5-axis task" 6-DOF robot redundancy; "semi-constrained"
Cartesian planning; ladder graph / layered graph IK path planning; Descartes ROS-Industrial planner; TrajOpt;
"offline programming" robotic welding collision avoidance; "pipe welding" robot positioner reachability;
"tool path" robot "configuration selection" dynamic programming; "kinematic redundancy" "machining" robot.

## 5. Output wanted
Annotated bibliography + synthesis per RQ, each claim marked Fact (source) / Assumption / Interpretation, and a list
of implications for the TST-002 decision on 29 Oct 2026. Implications that change the design go to
`docs/generated/DES-001_change_proposal.md` for team review — not straight into code.
