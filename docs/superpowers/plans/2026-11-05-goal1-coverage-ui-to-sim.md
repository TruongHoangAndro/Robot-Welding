# Goal 1 — weld-path coverage by robot + positioner, Web UI to simulation — Master Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Each task below is assigned to one agent; the agent writes its own TDD steps in its branch plan before coding.

**Goal:** On the golden part 564082, a person imports the STEP in the browser, picks J1 and J2, presses Plan, and
watches the robot and the rotary positioner follow both seams in simulation (Web playback and mock hardware in RViz) —
kinematics only, collision not checked (CP-58).

**Architecture:** Existing chain, collision switched off: M1 import + D20 (done) → M2 seam picking → D04 (part frame)
+ derived `seam_axis.json` (positioner frame, CP-58 decision 2) → M4 kinematic planner (ψ × 8 IK branches, ladder DP,
Mode C / Mode B) → D07 → M6 7-joint program D09 → playback (`awm_web` S4-lite) and replay on mock hardware.
`awm_api` runs the conda / ROS CLIs as subprocesses (T2.7 runner). Head TCP from the Fusion head CAD (`head.yaml`).

**Tech Stack:** pythonocc (conda, ADR-15), Python 3 / rclpy, MoveIt 2 only for FK cross-checks, ur-analytic-ik,
FastAPI, React + react-three-fiber + urdf-loader, ros2_control mock hardware.

**Spec:** CP-58 (Goal 1 / Goal 2, seam in the positioner frame), CP-53 (arcs entered by the Engineer), ADR-11 (Modes
B / C, no simultaneous motion during emission), DES-001 Rev 0.5 §4.3 steps 1–2, 4–6, §5.2, §5.3, §7.4; PLAN.md T1.2,
T1.4, T1.6, T2.2, T2.4, T2.5, T2.7, T3.1, T3.2; contracts `docs/generated/contracts/T0.5_contract_freeze_v2.md`, D20.

## Global Constraints

- Simulation only (mock hardware). No real robot, PLC, LightWELD, laser; no IP addresses; no laser commands.
- **Collision is not evaluated** in Goal 1. Every D07 / D09 / report carries `collision_checked: false`; the UI shows
  "kinematic only — collision not checked"; every release gate refuses such a program (DES-001 rule 4 unchanged).
- Still enforced in Goal 1: 100 % of points reachable or the seam / arc is rejected with its failing points (no
  partial path, no silent fallback); one IK branch per weld segment; |Δq| ≤ 5° per step (F1 pitch ceiling, CP-53);
  joint limits; wrist-3 ±180° travel per seam, ±270° absolute (A48); singularity margins (F6 values).
- Positioner: 1 axis, 360°, limits from `awm_description/config/positioner_limits.yaml` (Assumption, OI-14); Mode C
  for seams coaxial with the axis (robot at rest, ω = travel speed / r); Mode B = index to an **Engineer-entered angle**,
  stop, weld one arc (B-12; no automatic angle search — R3). No robot + positioner motion during a weld segment.
- Head: `head_model:=hcad`, `tool0_T_tcp` from `cell/model/head/head.yaml` as modelled (CP-58).
- Units mm / deg / s in project data and messages; m / rad only through `awm_core.units`.
- No part data, seam ids or poses in source; no absolute paths; one config source.
- `docs/reference/**`, `docs/LLM_Wiki/**` read-only; CAD files are inputs, never committed.
- ≤ 2 agents at a time (Truong 2026-11-03). ROS only via `scripts/dx`. Never push or merge.

## Review Focus

1. `seam_axis.json` stale after the placement or the seam changes → planner refuses (hash check), UI says "re-plan".
2. Seam direction / start point changed in S2 → φ order and arc split follow; Mode C ω sign follows the direction.
3. Coaxial test tolerance: J1 must be detected as coaxial (r spread and axis angle limits as parameters with
   Assumption defaults), J2 must not.
4. Mode B with entered angles leaving a gap or overlap < `arc_overlap_mm` → rejected with the uncovered s range.
5. Browser playback shows the same joint values the program contains (no re-interpolation drift): FK of the played
   q matches D07 TCP poses ≤ 0.01 mm.

---

## Tasks (order = waves; two agents per wave)

| # | Wave | Agent | Task | Done when |
|---|---|---|---|---|
| G1.1 | 1 | cell-sim | `head_model:=hcad` description: head + mount **visual** meshes from `head.yaml` folder, `tcp_laser` from `tool0_T_tcp_mm` (nozzle length arg), same link names as H-BOX (collision geometry parametric, unused in Goal 1); `sim_mock` arg; launch smoke test | test: `tcp_laser` in tool0 = `head.yaml` ≤ 1e-6 mm; RViz screenshot |
| G1.2 | 1 | seam-process | T2.2: picked edge ids (either solid) → joint edge → D04 v2 `seams.json` (direction, start point, `arcs`, `arc_overlap_mm`, `positioner_mode` auto / C / B, `positioner_angles_deg` per arc); sampling at ≤ 1 mm with joint-aware normals (S.1); new schema `v2/seam_axis` + writer of `seam_axis.json` (CP-58 d2) from `placement.json` + `frames.yaml`; CLI | tests: J1 r spread ≤ 0.01 mm at 13.33 mm, J2 not coaxial; part → axis → part round trip ≤ 1e-6 mm; stale hash refused |
| G1.3 | 2 | planner | `awm_path plan --job --collision off`: reads `seam_axis.json`, head and limits from the description; Mode C (one robot pose valid for the whole turn, positioner segment) / Mode B (per arc at the entered θ: ψ × 8 branches, ladder DP) with collision checks disabled by a recorded switch; D07 v2 with per-point status (ok / ik / limit / singularity / wrist3 / step) and `collision_checked: false`; reach map per θ (information for the Engineer, not a choice) | golden part: J1 Mode C 100 %; J2 Mode B with 2–3 entered angles 100 % or rejected with failing s ranges; reproducible |
| G1.4 | 2 | web | T3.1 subset + T2.7 runner: `awm_api` `/api/v1` jobs (STEP upload → `import` + `export-view` in the conda process), `/jobs/{id}/view`, `/seams` GET / PUT (+ `seam_axis` build), `/plan` (async run, progress over WebSocket), `/runs/{id}`, `/simulation`; login stub with roles; T3.2 S1-min (job list / create) and S2 (pick edge → joint edge, attributes incl. arcs and positioner angles, seam frame + beam axis drawn) | API tests with FastAPI client; S2 scripted browser run on the golden part |
| G1.5 | 3 | program-verify | T2.5 (Goal 1 subset): D09 v2 7-joint program from D07: approach / retract 40 mm along the beam, Mode B index moves with the robot parked, joint-interpolated air moves (unchecked, labelled), time parameterisation at the WPS travel speed in the part frame (A47; Mode C ω = v / r), weld events only at rest; `collision_checked: false`; release gate refuses it | tests: TCP speed error < 5 % on weld segments; events at rest; refusal test |
| G1.6 | 3 | web | T3.2 S4-lite: playback of D09 (robot via urdf-loader with the hcad head, positioner + part + cell view), scrubber, point colours by status, banner "kinematic only — collision not checked", cycle time | scripted browser run; FK check of played q vs D07 ≤ 0.01 mm |
| G1.7 | 4 | cell-sim | Replay D09 on `sim_mock` (robot `joint_trajectory_controller` + positioner controller, segment by segment, start-state check) with a CLI; used by `/simulation` "run in RViz" | launch test: golden part J1 + J2 replayed, final `/joint_states` = last D09 point |

**Acceptance (Goal 1 done):** Truong runs the golden part from the browser (import → pick J1, J2 → plan → playback)
and in RViz, without a terminal command except the launches; J1 and J2 covered 100 % (or J2 rejected with failing
points, then the Engineer changes the entered angles / arcs in S2 and re-plans).

**Base branch:** GotoCode after the lead merges (on Truong's choice) `ws-s4c-checks` → `ws-s5c-cp56-tcp` (planner
core, Mode B helpers, TCP from the description) and `ws-s5c-hcad-head` (`head.yaml` export). Goal 2 (collision) resumes
from those branches: S.5c Tasks 2–3, G2 pose Z5, CPU-parallel sweep, CP-57.
