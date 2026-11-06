# G1.3 — kinematic synchronised 7-axis planner (collision off) — branch plan

> Agent: planner · Branch `g1-3-sync-planner` (GotoCode f200f5e + G1.1 + G1.2) · Master plan:
> `2026-11-05-goal1-coverage-ui-to-sim.md` (task G1.3). Spec: CP-58, CP-59, CP-53 (arcs, F1 pitch ceiling),
> DES-001 Rev 0.5 §4.3 steps 1, 3–5, A41–A49; TST-002 Rev 0.3 draft §5.1 F1, F2, F3, F6, F7, F11.

## Design (no architecture change)

* **Geometry.** `seam_axis.json` (G1.2) gives every seam point and seam frame in `positioner_flange` at θ = 0.
  `base_link_T_point(θ) = base_link_T_world · world_T_positioner_axis · Rx(θ) · flange_T_point` (CP-58 d2).
  Cell frames from the robot description rendered from `awm_cell_v2.urdf.xacro` (args from `frames.yaml`, the same
  mapping as `sim_mock`); the URDF chain is checked against `frames.yaml` (one source, refuse on disagreement).
  `tool0_T_tcp` from `head_from_urdf` (H-CAD / H-BOX). Joint limits from the URDF, positioner limits from
  `positioner_limits.yaml` (Assumption, OI-14), cross-checked with the URDF `positioner_joint` limit.
* **State per point** = (θ, ψ, IK label); unwrapped 7-joint vector carried along the DP.
  * `sync`: θ lattice in the *world angle* α = φ + θ: α_k = k·δ (δ = `theta_step_deg` ≤ Δq_max, default 5°), so
    per point θ_k = k·δ − φ_i is a grid of spacing δ inside the limits (unwrapped, may exceed ±360°). Same k on two
    consecutive points keeps the point at the same angle about the axis (θ follows the seam).
  * `C`: robot at rest (one robot state), θ_i = α₀ − φ_i; only coaxial seams (`coaxial.is_coaxial`).
  * `B`: θ = the Engineer-entered arc angle (B-12), robot alone over the arc.
  * ψ grid 0–350° step 10° (`psi_case free`), 8 analytic IK branches (`ur-analytic-ik`), states beyond joint limits
    or inside the F6 singularity margins dropped. IK is computed once per θ class mod 360° and reused for θ ± 360°.
* **Ladder DP** (F2, F3, F7): edges only inside one label, |Δq| ≤ Δq_max on all 7 joints (θ included); exact edge
  set (θ offsets and ψ offsets bounded by the rotation argument Σ|Δq_j| ≥ rotation angle). Cost (CP-59, weights
  = parameters, Assumption defaults, recorded): w_dq·Σ₇|Δq| + w_theta·|Δθ| + w_wrist3·|Δq6| + w_up·∠(seam Z, world +Z).
  Tie-break: cost rounded 1e-9, then predecessor index (θ, label, ψ order). Joint limits / wrist 3 (F11: ±270°
  absolute, ±180° travel) on the unwrapped result; if the optimum breaks F11 the DP is re-run inside conservative
  wrist-3 windows (sufficient, never accepts a violation).
* **Pitch (CP-53 F1).** Start at the seam_axis spacing (≤ 1 mm); if the DP fails on a step break, halve the pitch
  (midpoints on the circle through the neighbouring samples, slerp of the frame; Assumption) down to the 0.05 mm
  floor; the pitch used is recorded.
* **Closed seams.** Arcs from `seam_axis.json` (overlap planned as explicit layers). The F3 closure (state at
  s = L vs s = 0) is computed and reported (robot Δq, Δθ), not enforced — the overlap layers already make the weld
  continuous; flagged to the lead.
* **Output** D07 v2 MINOR 2.1: θ, q (6) per point, status, `collision_checked: false`, `min_clearance_mm: null`,
  per-arc planning stats, rejected arcs with failing s ranges; context record (git, hashes, head, description).
  `timed` = weld segment at the travel speed relative to the part (A47; Mode C `positioner_weld`), joint speeds
  vs limits recorded (not an acceptance rule in G1.3; reported).

## Steps (TDD)

- [x] 1. Contract MINOR (lead-approved): `common` positioner_mode + `sync`; `failing_point.status`; `v2/path` 2.1
  (`collision_checked`, point `status`, nullable `min_clearance_mm`, arc `planning`, rejected `failing_ranges`,
  planner `params`); example + `test_jsonio`.
- [x] 2. `core/batch_kin.py`: vectorised UR5e DH FK, labels and F6 metrics; test against `UrAnalyticKinematics.fk`
  and `states.label_and_metrics`.
- [x] 3. `core/cell_urdf.py`: cell frames, joint and positioner limits from a URDF; agreement check with frames.yaml.
- [x] 4. `core/sync.py`: rotation model (test: point at θ vs FK of the planned q), state generation, DP, modes
  sync / C / B, pitch refinement, failing ranges. Tests: synthetic coaxial ring (sync and C 100 %), Mode B uses only
  the entered θ, |Δq| ≤ 5° on 7 joints, FK ≤ 0.01 mm / 0.01°, determinism.
- [x] 5. `core/d07.py` + `cli.py` (`python3 -m awm_path.cli plan`, entry `awm_path_cli`): progress lines, refusal of
  `--collision on` and stale `seam_axis.json`, schema validation, identical payload on re-run.
- [x] 6. Golden run (env `AWM_SPIKE_REPO`, copy of the spike job): J1 sync + C, J2 sync; run record in
  `docs/generated/goal1/`.
- [x] 7. `colcon test --packages-select awm_core awm_path awm_seam`; report.
