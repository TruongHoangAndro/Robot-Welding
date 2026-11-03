---
title: Ladder-graph weld path planning (collision-free, tool-yaw redundancy)
type: concept
domain: robot-collision-ai
project: APLSMN601
status: proposed            # DES-001 ADR-12 "Proposed — decide after TST-002"
updated: 2026-10-27
sources:
  - APLSMN601-DES-001 Software Architecture Rev 0.4 (§4.3, §5.4, ADR-12)
  - APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.2
  - ROS-Industrial training "Motion Planning with Tesseract" (Humble)
  - arXiv 2508.04146 (cuRobo for extended-DOF industrial robots)
  - Robotmaster product note (MMS Online)
tags: [collision-avoidance, weld-path, ur5e, ik, moveit2, fcl, dynamic-programming, laser-welding]
---

# Ladder-graph weld path planning

## Summary
Method chosen (proposed) for APLSMN601 to turn a picked seam into a robot path that never hits the part, fixture,
clamps or the weld head itself. Collision avoidance and path finding are solved **together**: for every point on the
seam, enumerate all robot poses that keep the beam on the seam, cross out the bad ones, then pick one pose per point so
that the robot moves continuously from the first to the last point. If no continuous chain exists the seam is rejected
and the failing point and colliding link are reported — the robot never "tries and hits".

Status: DES-001 Rev 0.4 **ADR-12 — Proposed, decide after TST-002**. Fallback: Tesseract planning stack (Descartes + TrajOpt + OMPL).

## Why it works (key idea)
- Laser / MIG welding constrains only **5 DOF** (TCP position + beam axis). The UR5e has 6 → **1 redundant DOF:
  rotation about the beam axis (tool yaw ψ)**.
- The UR5e also has up to **8 IK solutions** (shoulder / elbow / wrist configuration) for the same TCP pose.
- Tolerances of the process (work / travel angle band) add more freedom.
- These freedoms are the "room" to avoid collisions. Planning = searching this room, not fixing a single pose.

Teach-pendant analogy (for engineers new to robotics):

| On the teach pendant | In the ladder-graph method |
|---|---|
| Jog to a point, store a waypoint | Discretise the seam every 1–3 mm |
| Rotate about the TCP to dodge the part | Sample ψ (36 values) |
| Change robot configuration (elbow up/down, wrist flip) | Try all 8 IK branches |
| Look whether something touches | FCL collision + clearance check on the 3D model |
| Re-teach from the start when stuck mid-seam | Graph search over the whole seam before committing |

## Algorithm (DES-001 §4.3)
1. **Discretise** the seam by arc length (1–3 mm). Seam frame: X = tangent, Z = bisector of face normals n1, n2
   (from B-Rep, pointing out of the joint), Y = Z × X.
2. **Nominal head orientation** from joint type + WPS; tolerance band per seam.
3. **Sample candidates** per point: ψ = 0–350° step 10° × 8 IK branches (= 288 states). Work / travel angle fixed in
   Phase 1 (sweep only in the pivot path).
4. **Filter** each state: IK exists → joint limits → singularity margin → collision (FCL) → clearance ≥ padding (3 mm).
5. **Ladder graph + dynamic programming**: layers = seam points, nodes = surviving states. Edges only between states
   with the same IK branch and a bounded joint step. Cost = joint motion + deviation from nominal angles
   + low-clearance penalty + wrist-3 travel (fiber / wire-feed conduit wrap). Shortest path = weld path.
6. **No path** → reject seam; report first unreachable point, reason (collision / reach / limit / singularity) and the
   link ↔ object pair. No partial path, no silent fallback (DES-001 rule 4).
7. **Approach / retract** 30–50 mm along the beam axis; **air moves** between seams with OMPL; all collision-checked.
8. Time parameterisation at WPS travel speed; store path in the `part` frame (mm, ADR-14).

## Collision model (DES-001 §5.4)
| Object | Representation | Rule |
|---|---|---|
| Part | Mesh from STEP, chord ≤ 0.1 mm near seams, `face_id` per triangle | ACM: only the TCP link may touch seam faces |
| Fixture, clamps | Simplified STL / convex hulls | 3 mm padding |
| LightWELD head + mount, fiber / wire conduit | Convex hulls / capsules; simplified model (TST-002 §3.1) until IPG CAD arrives | Fiber bend radius ≥ 50 mm |
| UR5e | ur_description collision meshes | — |
FCL works on meshes only — never on B-Rep.

## Libraries
- **MoveIt2 `move_group` + FCL**: planning scene, `check_state_validity`, distance queries. **OMPL** for air moves.
- **IK**: analytic UR IK with all 8 solutions — `ur-analytic-ik` (PyPI) or a port of `ur_kin.cpp`.
  `pick_ik` returns one solution (not enough); `ur_kinematics` is ROS 1 only; `opw_kinematics` does not fit the UR wrist.
- **UR packages**: `ros-jazzy-ur` (ur_robot_driver, ur_description, ur_moveit_config). UR ships **no** collision-avoidance library;
  the controller's force limit / safety planes only protect after contact.

## Parameters (Phase 1 defaults — tune in TST-002)
| Parameter | Value | Note |
|---|---|---|
| Seam point pitch | 1–3 mm (spike: 2 mm) | |
| ψ step | 10° | refine locally if no path |
| IK branches | 8 | no branch change inside a seam |
| Max joint step between points | spike: ≤ 5° | |
| Clearance padding | 3 mm (nozzle/head target ≥ 5 mm in spike) | |
| Approach / retract | 30–50 mm | along beam axis |
| Wrist-3 window | ±180° from seam start | conduit wrap proxy |
| Planning time target | ≤ 30 s per seam | TST-002 C5 |

## Strengths and limits
**Strengths** — global within the grid (no greedy dead-ends), deterministic, explainable (every rejected state has a
reason), fits the existing MoveIt2 stack of the Tac demo.

**Limits** (Interpretation):
- Discrete grid can miss narrow passages; path slightly "stepped".
- Adding work/travel sweep multiplies states (36 × 8 × 5 × 5 = 7 200 per point).
- Collision checked at waypoints; clearance penalised, not maximised.
- Cannot fix seams that are out of reach for every ψ — needs part placement / seam split / positioner.

## Better options built on top (evaluated 2026-10-26)
| Option | What it adds | When |
|---|---|---|
| A. Two-stage: ladder graph → continuous optimisation (Descartes → TrajOpt style) | Max clearance, smooth path, continuous ψ and angle tolerances. Tesseract does this natively (ψ sampled, TrajOpt Z-rotation weight 0) | Phase 2, or if clearance is marginal |
| B. GPU-dense sampling (cuRobo / cuMotion) | Much faster batched IK + collision → finer grid. Paper: ~45 ms vs ~1 200 ms planning (pick & place, self-reported). Robot approximated by spheres; license to confirm; needs NVIDIA GPU on IPC | Only if TST-002 C5 fails |
| C. Outer loop: part placement + seam split (+ positioner later) | Reachability-driven choice of part pose / seam segments; ladder graph scores each option. Targets 360° pipe seams | Phase 1 after TST-002 if C2 / C8 fail |
| Commercial OLP (e.g. Robotmaster) | Built-in torch-orientation optimisation, singularity / collision / limit avoidance, positioners | Benchmark; price and UR5e support to confirm |

## Demo / teaching material
`4 DRAWINGS AND DESIGN EVALUATION\4. SOFTWARE\4. Collision\APLSMN601_P2_Demo_v2.html` — interactive **concept demo**
(2D, VI/EN). Illustrative only: simplified 2D model, not a simulation of the real cell.

## Open questions
- TST-002 result (Go / Pivot / No-Go) and C8 360° reach → decides ADR-12 and OI-20 (positioner).
- Real LightWELD head + mount CAD and clamp geometry (DES-001 OI-15).
- pythonocc-core version mismatch: workspace env 8.0.1 / Py 3.10 vs ADR-15 7.9 / Py 3.12.
- `descartes_light` / `tesseract_ros2` support on Jazzy — not yet checked.
- cuRobo license for commercial use — not yet confirmed.

## Related
[[ur5e-robot-functions]] · [[laser-welding-lightweld]] · [[robot-collision-ai]] · [[tst-002-collision-spike]] · [[des-001-software-architecture]]
<!-- link targets assumed; adjust to the page names defined in PROJECT.md -->

## Sources
- APLSMN601-DES-001 Rev 0.4 — §4.3 M4 algorithm, §5.4 collision model, ADR-12 (Drive master, 4…\4. SOFTWARE)
- APLSMN601-TST-002 Rev 0.2 — scope, simplified head model, criteria C1–C8
- https://industrial-training-master.readthedocs.io/en/humble/_source/session8/Motion-Planning-with-Tesseract.html
- https://arxiv.org/html/2508.04146v2
- https://github.com/NVLabs/curobo
- https://www.mmsonline.com/products/cam-software-eases-programming-of-welding-robots
- https://pypi.org/project/ur-analytic-ik
