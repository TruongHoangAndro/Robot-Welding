# Collision-free weld path planning — first-pass literature review

Date: 2026-10-27 · Brief: `01_research_brief.md` · Prepared by Claude (Opus 5.5) for Truong Hoang
Status: **first pass, not an ARS pipeline run.** The ARS plugin was installed in this folder during this session but is
loaded only by a new Claude Code session opened here. This pass follows the ARS `lit-review` structure (search →
source verification → synthesis) by hand, with web search only.

## How to read this
- **Fact** = stated by a source I opened (page, README, or API response). **Fact (abstract)** = taken from an abstract
  or search summary, full text not read. **Interpretation** = my reading. **Assumption** = not verified.
- Publisher pages (ScienceDirect, MDPI, Springer, Cambridge) blocked automated access, so most papers are at
  abstract level. Get the full texts before quoting them in DES-001 or the TST-002 Rev 0.3 report.
- Review form: narrative review for an engineering decision. No systematic search or screening was done.

## Summary
1. The ADR-12 method (sample the free rotation about the tool axis × IK solutions per waypoint, then pick one state
   per waypoint by shortest-path search) is a **recognised, published pattern** for 5-DOF tasks on 6-DOF arms in
   welding, milling and additive manufacturing. Descartes (ROS-Industrial) is the open-source implementation of the
   same idea. Nothing found argues against it for a single seam with ~100–500 waypoints.
2. The known weakness is **scaling**: the graph grows with waypoints × samples, and the discrete result needs
   smoothing / time parameterisation afterwards. TST-002 C5 (≤ 30 s) is the place where this would show.
3. For the **Pivot path (C2 fails)**, the literature uses the same levers as TST-002 §7: relax work / travel angle,
   use extra DOF (positioner, movable base, rotating pipe), or change the setup. A Robotica paper on intersecting pipe
   welding compares fixed base vs movable base vs rotating pipe directly — the best single input for OI-20.
4. **360° pipe seams without positioner (C8)**: no study found that shows this is generally feasible with a fixed
   6-DOF arm; published pipe-welding cells mostly use a positioner. This is absence of evidence, not proof.
5. **Fact-check of library assumptions** — mostly confirmed, with two new points:
   pick_ik is **deprecated** by PickNik, and the Tesseract ROS 2 wrapper README lists **Foxy and Humble only, not
   Jazzy**. `ur-analytic-ik` warns that default DH parameters give **1–2 mm FK error** on a real robot.

## RQ1 — Is "ψ sampling × IK branches + graph search" the established method?
| # | Finding | Grade | Source |
|---|---|---|---|
| 1.1 | For a 5-DoF tool path on a 6-DoF robot, the redundancy is defined as the rotation about the tool axis; robotic milling commonly generates the 5-axis path first, then resolves this redundancy. | Fact (abstract) | [Co-optimization…, arXiv 2409.13448](https://arxiv.org/html/2409.13448) |
| 1.2 | Welding robots have this angular redundancy, it is usually set by hand, and a sampling-based welding planner uses it to avoid obstacles while keeping the seam continuous. | Fact (abstract) | [RCIM, S0736584518306392](https://www.sciencedirect.com/science/article/abs/pii/S0736584518306392) |
| 1.3 | Descartes is a ROS-Industrial planner for "semi-constrained" Cartesian paths: 5 DOF fixed, rotation about the tool left free. Alpha 2015, NIST + RIC-Americas. | Fact | [ROS-I news 2015](https://rosindustrial.org/news/2015/6/26/descartes-path-planner-alpha-release), [ROSCon 2015 slides](https://roscon.ros.org/2015/presentations/ROSCon_Descartes.pdf) |
| 1.4 | An evaluation of Descartes for arc-welding robots (Hasselt / KU Leuven, ETFA 2017, DOI 10.1109/ETFA.2017.8247616) reports that results depend on the cost function minimised by the graph search. | Fact (abstract) | [CORE record](https://core.ac.uk/works/73324968), [UHasselt record](https://documentserver.uhasselt.be/handle/1942/26480) |
| 1.5 | Graph search over candidate configurations is used in machining (directed graph of robot configurations + improved A*, with collision and joint-limit checks). | Fact (abstract) | [RCIM 2025, S0736584525001681](https://www.sciencedirect.com/science/article/abs/pii/S0736584525001681) |
| 1.6 | Y-joint welding: optimal torch path found with functional redundancy and a **beam search** minimising joint movement. | Fact (abstract) | [IJAMT, 10.1007/s00170-016-9684-z](https://www.doi.org/10.1007/S00170-016-9684-Z) |
| 1.7 | Dynamic programming over path inversion with the IK solution index as a discrete state, unified with time parameterisation (Panda robot). | Fact (abstract + search summary) | [Ferrentino et al., IEEE T-ASE 2023, arXiv 2207.05622](https://arxiv.org/abs/2207.05622) |
| 1.8 | Drawbacks: the search graph grows combinatorially with waypoints and redundancy dimension; large waypoint counts → low efficiency and high memory; timing constraints (velocity, acceleration, jerk) need a separate speed-planning step. | Fact (abstract) | [arXiv 2606.29766](https://arxiv.org/html/2606.29766), [arXiv 2409.13448](https://arxiv.org/html/2409.13448) |
| 1.9 | Alternatives: gradient-based / optimisation planners with differentiable collision models (TrajOpt family, constrained gradient projection). | Fact (abstract) | [arXiv 2606.29766](https://arxiv.org/html/2606.29766), [ROSCon 2018 Tesseract + TrajOpt](https://roscon.ros.org/2018/presentations/ROSCon2018_Tesseract_and_TrajOpt.pdf) |

**Interpretation.** ADR-12 is the same family as Descartes and the milling "candidate posture" methods. Its additions
(8 analytic branches, clearance cost, wrist-3 travel cost, hard rejection when any waypoint has no valid state) fit
the project rules and are not unusual. With 36 ψ × 8 branches = 288 states per waypoint and a 2 mm pitch, a 300 mm seam
has ~150 layers and ~12 M edges if every pair is connected — feasible in DP, but the collision checks
(~43 k `check_state_validity` calls) are the likely bottleneck for C5, not the graph search. Bounding |Δq| per step
(TST-002 C3) also prunes most edges.

## RQ2 — What do published systems do when no valid state exists?
| # | Finding | Grade | Source |
|---|---|---|---|
| 2.1 | Intersecting-pipe welding with a redundant system: three setups compared — fixed base, movable base, rotating pipe; extra DOF used to improve weld quality. | Fact (search summary) | [Ghariblu & Shahabi, Robotica 2019](https://cambridge.org/core/journals/robotica/article/path-planning-of-complex-pipe-joints-welding-with-redundant-robotic-systems/F097BC9C252DB2B1AFF8995D41D4E600) |
| 2.2 | Automatic programming for intersecting pipes built around a 1-DOF positioner rotating the main pipe. | Fact (search summary) | [ResearchGate 277604382](https://www.researchgate.net/publication/277604382_Automatic_programming_for_industrial_robot_to_weld_intersecting_pipes) |
| 2.3 | Redundancy optimisation of a 6R robot for 5-axis milling with singularity, joint limits and collision as combined objectives. | Fact (abstract) | [Xiao & Huan, Prod. Eng. 2012](https://link.springer.com/article/10.1007/s11740-012-0362-1) |
| 2.4 | Joint motion planning for redundant industrial robots in remote laser welding, using redundancy to cut cycle time. | Fact (search summary) | [Erdős, Kovács, Váncza — Fraunhofer record](https://publica.fraunhofer.de/entities/publication/bf185a73-23a4-480f-aa5d-e89efed3670f) |

**Interpretation.** The order in TST-002 §7 (angle relaxation → tilted mount → part orientation / clamps / base →
split seam → positioner) matches the levers in the literature. Item 2.1 is the most direct input for OI-20 because it
compares the fixed-base case against extra-DOF cases on the same pipe joint. A tilted head mount is not covered by the
sources found; search for it separately ("tool mounting angle optimisation robot welding").

## RQ3 — 360° pipe seams without a positioner
| # | Finding | Grade | Source |
|---|---|---|---|
| 3.1 | Intersecting-pipe seams are 3D curves with continuously changing curvature and torsion, forcing continuous torch re-orientation. | Fact (search summary) | [Applied Sciences 2026, 16(6) 2974 — review](https://www.mdpi.com/2076-3417/16/6/2974) |
| 3.2 | Published intersecting-pipe cells mostly assume a positioner (items 2.1, 2.2). | Fact (search summary) | as above |
| 3.3 | No source found that reports a full 360° intersection seam welded by a fixed 6-DOF arm alone, with reach analysis. | Search result | — |

**Interpretation.** For a fixed UR5e the limits are reach on the far side of the pipe and wrist-3 wrap
(the seam frame turns 360° about the pipe axis). TST-002 C8 measures exactly this, so the spike result will be stronger
evidence than the literature. Elbow 564082 seams are pipe-to-elbow butt joints (coaxial), which are easier than saddle
joints — check this when choosing the C8 seam. **Assumption** until checked on the STEP model.

## RQ4 — Fact-check of library assumptions (CLAUDE.md, ADR-12)
| Claim in project docs | Result | Source |
|---|---|---|
| `ur-analytic-ik` returns all UR IK solutions at once | **Confirmed** ("finds all solutions at once", no initial guess). The README does not state the count 8 — verify by test. Experimental; API may change. | [README](https://github.com/Victorlouisdg/ur-analytic-ik), [PyPI](https://pypi.org/project/ur-analytic-ik) |
| — (new) | **Warning:** uses default DH parameters; tests show **1–2 mm FK difference** vs a real robot's factory calibration. | [README](https://github.com/Victorlouisdg/ur-analytic-ik) |
| `pick_ik` returns one solution — not enough for the sweep | **Consistent.** Hybrid gradient-descent local + evolutionary global solver (numeric, one result per request). README does not state the count. **New:** PickNik has deprecated pick_ik, basic maintenance only. | [README](https://github.com/PickNikRobotics/pick_ik) |
| `opw_kinematics` does not fit the UR wrist | **Consistent.** Library targets "parallel bases and spherical wrists"; UR has an offset (non-spherical) wrist — Interpretation from UR kinematics. | [README](https://github.com/Jmeyer1292/opw_kinematics) |
| Tesseract + tesseract_planning (Descartes, TrajOpt, OMPL) as fallback | `tesseract_planning` implements OMPL, TrajOpt, TrajOpt IFOPT and Descartes; ROS-agnostic core, active (push 2026-10-14). **`tesseract_ros2` README lists Foxy and Humble only — Jazzy not stated** (last push 2026-09-26). | [tesseract_planning](https://github.com/tesseract-robotics/tesseract_planning), [tesseract_ros2](https://github.com/tesseract-robotics/tesseract_ros2) |
| Descartes | ROS 1 `descartes` unmaintained (index.ros.org; last push 2023-06). Successor `descartes_light` (SwRI) active (push 2026-08-28), ROS-agnostic, used inside tesseract_planning. | [index.ros.org](https://index.ros.org/p/descartes/), [descartes_light](https://github.com/swri-robotics/descartes_light) |

## Implications for the TST-002 decision (29 Oct 2026)
These are proposals for the team, not design changes. Items that change DES-001 go to
`docs/generated/DES-001_change_proposal.md`.
1. **Keep ADR-12 as proposed.** The literature supports the method; the spike decides feasibility on part 564082.
2. **Pivot fallback cost is higher than DES-001 implies.** Tesseract has no stated Jazzy support in its ROS 2 wrapper.
   `descartes_light` + `tesseract_planning` can be used ROS-agnostic, but that is integration work. Record this as a
   risk under ADR-12 / ADR-10 (Jazzy).
3. **Kinematic calibration and accuracy (AD5 ±0.1–0.2 mm).** Default UR DH parameters can be 1–2 mm off in FK. The
   offline planner, the MoveIt URDF and the replayed joint trajectories must use the robot's own calibrated kinematics.
   ur_robot_driver has a calibration-extraction step for this — **Assumption, verify on the Jazzy driver docs**.
   Not a Sim-1 issue; must be in the hardware-stage plan (A51 / D08).
4. **pick_ik is deprecated.** It is still in the demo MoveIt config. For the ψ sweep it is replaced by analytic IK
   anyway; for MoveIt's own IK plugin, note the deprecation in the change proposal.
5. **C5 planning time.** Expect collision checks, not the DP, to dominate. Measure them separately in `candidates.csv`.
6. **OI-20 input.** Read Ghariblu & Shahabi (Robotica 2019) in full before the positioner decision.

## Gaps and next steps
- Full texts needed: items 1.2, 1.4, 1.6, 2.1, 3.1 (company library or author copies).
- Not covered yet: tilted tool-mount optimisation; fiber / cable wrap limits on wrist 3; coaxial pipe-to-elbow seams.
- Next ARS run (open a new Claude Code session in this folder):
  - `/ars-3w` on items 1.2, 1.4, 1.6, 2.1, 1.7 → WHY / HOW / WHAT table.
  - `/ars-citation-check` on this file before any item is quoted in a project document.
  - deep-research `fact-check` on Implication 3 (UR calibration on ur_robot_driver Jazzy).

## Sources
- https://www.sciencedirect.com/science/article/abs/pii/S0736584518306392
- https://www.sciencedirect.com/science/article/abs/pii/S0736584525001681
- https://www.sciencedirect.com/science/article/abs/pii/S073658452030301X
- https://arxiv.org/html/2606.29766
- https://arxiv.org/html/2409.13448
- https://arxiv.org/abs/2207.05622
- https://arxiv.org/abs/1909.06570
- https://www.doi.org/10.1007/S00170-016-9684-Z
- https://link.springer.com/article/10.1007/s11740-012-0362-1
- https://cambridge.org/core/journals/robotica/article/path-planning-of-complex-pipe-joints-welding-with-redundant-robotic-systems/F097BC9C252DB2B1AFF8995D41D4E600
- https://www.researchgate.net/publication/277604382_Automatic_programming_for_industrial_robot_to_weld_intersecting_pipes
- https://www.mdpi.com/2076-3417/16/6/2974
- https://publica.fraunhofer.de/entities/publication/bf185a73-23a4-480f-aa5d-e89efed3670f
- https://core.ac.uk/works/73324968
- https://documentserver.uhasselt.be/handle/1942/26480
- https://rosindustrial.org/news/2015/6/26/descartes-path-planner-alpha-release
- https://roscon.ros.org/2015/presentations/ROSCon_Descartes.pdf
- https://index.ros.org/p/descartes/
- https://roscon.ros.org/2018/presentations/ROSCon2018_Tesseract_and_TrajOpt.pdf
- https://github.com/tesseract-robotics/tesseract_planning
- https://github.com/tesseract-robotics/tesseract_ros2
- https://github.com/swri-robotics/descartes_light
- https://github.com/Victorlouisdg/ur-analytic-ik
- https://pypi.org/project/ur-analytic-ik
- https://github.com/PickNikRobotics/pick_ik
- https://github.com/Jmeyer1292/opw_kinematics
