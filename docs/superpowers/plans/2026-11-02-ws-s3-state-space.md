# WP-S S.3 — M4 core: state space, candidate sweep and ladder DP Implementation Plan

**Goal:** Build the pure part of the TST-002 planner: per seam point and per tool yaw ψ, all IK states with their
configuration label, unwrapped joint vectors, singularity metrics and rejection reasons; then the ladder-graph dynamic
programme that returns one continuous path or the failing points. Scene validity, clearance and contact (S.4) are
inputs here, supplied by a callable; S.3 is fully testable on synthetic ladders without ROS.

**Spec:** `PLAN.md` §3 WP-S row S.3; TST-002 Rev 0.3 DRAFT §3.3 (ψ cases), §5.1 F1–F3, F6, F7, F11
(`docs/generated/drafts/APLSMN601-TST-002_Collision_Avoidance_Spike_Rev0.3_DRAFT.md`); DES-001 Rev 0.6 DRAFT §4.3 steps
1, 3–5 and board decision B-03 (`docs/generated/arch_review_board_2026-10-30.md`); S.2 review follow-ups R1, R2, R4
in `PROGRESS.md`. Background: `raw/ladder-graph-weld-path-planning.md`.

**Builds on (S.2, merged):** `awm_core.units`; `awm_path.core.kinematics` (`KinematicsBackend`, `UrAnalyticKinematics`,
`fk_tcp`, `ik_tcp`, `wrap_deg`, `pose_error`); `awm_path.core.scene` (`StateCheck`, `SceneBackend`).

## Global Constraints

- Agent `planner`, worktree `.worktrees/planner` on branch `ws-s3-state-space` (create with
  `scripts/wt new planner ws-s3-state-space`; ROS_DOMAIN_ID 43). Build / test only with `.worktrees/planner/scripts/dx`.
  Commit prefix `awm_path:`. Write scope `src/awm_path/` only.
- `awm_path.core` stays pure: no rclpy, no ROS messages, no OCC. Public values mm and degrees; conversion only through
  `awm_core.units`. Joint order as S.2 (6 robot joints; the positioner is not part of S.3).
- No part data, no cell values in source: joint limits, Δq_max, pitch limit, singularity thresholds, ψ cases and cost
  weights are parameters of a frozen dataclass `PlannerParams` (values of TST-002 §5.1 as documented defaults, marked
  Assumption where TST-002 says so). D16 `cell.yaml` will supply them later (T0.1).
- Frozen definitions are not tuned to make a test pass. Determinism: same inputs → byte-identical outputs (no
  unordered set iteration, stable sorts, tie-break F7).
- A seam is accepted only at 100 %: the DP never returns a partial path; failure returns the failing layer(s) with the
  reasons counted per point.

## Review Focus

1. Configuration label = function of the joint values only (F2), 8 labels; on a non-singular pose the ≤ 8 IK solutions
   carry distinct labels; label stable under 2π offsets of any joint.
2. Unwrapping: the 2π offsets of every joint whose limit range exceeds 360° are enumerated at the first layer only;
   later layers take, per IK solution, the representative nearest the predecessor (|Δq| ≤ 180° per joint); limits
   checked on unwrapped values.
3. An edge never joins two labels; |Δq| ≤ Δq_max on every joint (unwrapped).
4. Closed seam: the overlap layer reconnects to the **same** first-layer state (F3); a path that closes on a 2π-shifted
   copy is not accepted.
5. Wrist 3 (F11): |q6| ≤ 270° absolute and |q6 − q6(start)| ≤ 180° along the path, both on unwrapped values. Any
   conservative grouping used for speed is documented and tested (it may reject, never accept, a violating path).
6. `ik` returning [] for a pose inside the reach envelope is reported as `no_ik_near_singular`, not `unreachable` (R2).

---

### Task 1: S.2 follow-ups R4 and R2

**Files:** `src/awm_path/test/test_core_kinematics.py`, `src/awm_path/awm_path/core/kinematics.py` (docstring only, if needed)

- [ ] R4 test `test_fk_tcp_composition_order`: `tool0_T_tcp` = translation (0, 0, 250) mm → for 50 seeded states
  `fk_tcp(q)[:3, 3] == fk(q)[:3, 3] + 250 * fk(q)[:3, 2]` within 1e-6 mm; and a rotated tool (15° about X) checked
  against an explicit `fk @ tool` product.
- [ ] R2 test documenting the library behaviour: at q5 = 0 exactly, some reachable poses give `ik(...) == []`
  (record the fraction found; the test asserts only that every returned solution is valid, never that the list is
  non-empty).
- [ ] Commit — `awm_path: tests pin fk_tcp composition order and document IK at q5 = 0 (S.2 R2, R4)`

### Task 2: configuration labels and unwrapping (R1, F2)

**Files:** create `src/awm_path/awm_path/core/states.py`; test `src/awm_path/test/test_core_states.py`

**Produces:**
- `config_label(q_deg) -> str`, three flags `S±E±W±` (shoulder, elbow, wrist) computed from joint values only. Use the
  standard UR analytic-IK branch definition: shoulder from the sign of the wrist-centre side with respect to q1, elbow
  = sign(sin q3), wrist = sign(sin q5). Document the exact formula; zero / boundary values map deterministically.
- `unwrapped_variants(q_deg, limits_deg) -> List[np.ndarray]`: all 2π-offset representatives inside the limits
  (deterministic order).
- `nearest_representative(q_deg, prev_unwrapped) -> np.ndarray`: per joint the 2π offset closest to the predecessor.
- `SingularityMetrics` (|sin q5|, |sin q3|, wrist-centre distance to the base Z axis in mm) and
  `singular_reason(metrics, params) -> Optional[str]` per F6.

- [ ] Tests: 500 seeded non-singular poses → `ik` solutions carry pairwise distinct labels and `len ≤ 8`; label of
  `q + 360·k` equals label of `q`; `unwrapped_variants` count matches the limit ranges (e.g. ±360° → 2 per joint,
  ±180° → 1); `nearest_representative` never moves a joint by > 180°; F6 thresholds at the boundaries.
- [ ] Commit — `awm_path: configuration labels, unwrapped joint variants, singularity metrics (S.3, R1, F2, F6)`

### Task 3: tool pose per point and ψ, candidate sweep

**Files:** create `src/awm_path/awm_path/core/sweep.py`; test `src/awm_path/test/test_core_sweep.py`

**Produces:**
- `PlannerParams` (frozen dataclass): `pitch_max_mm` 3.0, `dq_max_deg` 5.0, joint limits (deg), F6 thresholds,
  wrist-3 limits (270 / 180), ψ cases `psi_free` (0–350 step 10) and `psi_wire` (centre, ±15, step 5; centre 0 =
  Assumption), cost weights (`w_dq`, `w_dev`, `w_clear`, `w_sing`, `w_wrist3`), `clearance_target_mm`.
- `tcp_pose(seam_frame_mm, psi_deg) -> np.ndarray`: `seam_frame · Rx(180°) · Rz(ψ)` so tool Z = beam axis = −seam Z,
  ψ = rotation about the beam axis, ψ = 0 ↔ tool X = seam X. Work / travel angle 0 (TST-002 §2: nominal).
- `pitch_points(...)` helper for F1: given point spacing and frame rotation rate, the pitch actually used and a check
  that Δs ≤ min(pitch_max, Δq_max / rate) (input: frames already sampled by the caller).
- `Candidate` (frozen): point index, ψ, label, unwrapped q (6), metrics, `reason` (None = kinematically valid; else
  `no_ik_unreachable`, `no_ik_near_singular`, `joint_limit`, `singular_wrist|elbow|shoulder`).
- `sweep(frames, tool0_T_tcp, kin, params, psi_values) -> List[List[Candidate]]` (layer per point), plus
  `write_candidates_csv(path, layers, extra_columns)` with the columns of TST-002 §4 (point, ψ, label, q1–q6, metrics,
  reason, plus scene columns filled by S.4).

- [ ] Tests: synthetic circular seam (r = 13.3 mm, 72 points) placed reachable in front of the robot → every point has
  valid candidates; tool Z of every candidate = −seam Z within 1e-9; FK of each candidate reproduces its TCP pose
  < 0.01 mm; a seam placed 2000 mm away → all `no_ik_unreachable`; a frame forcing q5 ≈ 0 → `singular_wrist`;
  deterministic output (two runs identical).
- [ ] Commit — `awm_path: tool pose per psi, candidate sweep and candidates.csv (S.3)`

### Task 4: ladder-graph DP

**Files:** create `src/awm_path/awm_path/core/ladder.py`; test `src/awm_path/test/test_core_ladder.py`

**Produces:**
- `NodeScore` from the caller per candidate (S.4 provides it later): `valid: bool`, `clearance_mm: float | None`.
- `solve(layers, scores, params, closed: bool) -> LadderResult` with `ok`, `path` (one Candidate per layer, unwrapped),
  `cost`, and on failure `failing_layers` + per-layer reason counts + `max_reached_layer`.
- Edges per F3 (same label, |Δq| ≤ Δq_max all joints, unwrapped via `nearest_representative`); first layer expanded
  with all `unwrapped_variants`; cost = `w_dq`·Σ|Δq| + `w_dev`·|ψ − ψ_nominal| + `w_clear`·max(0, target − clearance) +
  `w_sing`·singularity penalty + `w_wrist3`·|Δq6|; tie-break F7.
- Closed seams: the last layer must connect by a valid edge to the chosen first-layer state itself (F3).
- Wrist 3 per F11 on unwrapped values (Review Focus 5).
- Efficiency: edges found per label with sorted / bucketed joint values, not all-pairs; report node / edge counts and
  wall time in the result. Target for the spike: J2-sized problem (~72 layers × 36 ψ × 8 labels × variants) well under
  C5 = 30 s excluding scene calls.

- [ ] Tests on synthetic ladders (no kinematics): unique feasible path found; label change never used even when cheaper;
  |Δq| bound respected; gap layer → `ok False` with that layer reported (no partial path); closed seam rejects a path
  that closes only on a 2π-shifted copy; wrist-3 travel > 180° rejected; tie-break order; determinism.
- [ ] End-to-end pure test: Task 3 synthetic circle → `solve(closed=True)` with all scores valid → `ok` and every step
  ≤ 5°; record node / edge counts and time.
- [ ] Commit — `awm_path: ladder-graph DP with closed-seam reconnection and wrist-3 limits (S.3, F3, F7, F11)`

### Task 5: verification and report

- [ ] `.worktrees/planner/scripts/dx "colcon build --symlink-install && colcon test && colcon test-result --verbose"` → 0
  failures (allowed skips: OCC-in-ROS-Python modules re-run in conda, `AWM_CELL_STEP not set`).
- [ ] Units scan on `src/awm_path/awm_path/core`: no scale literals or `radians`/`degrees` outside `awm_core.units`.
- [ ] Do not edit `PROGRESS.md`; do not merge. Report per AGENT_TEAM.md §3, including the Task 4 timing numbers.
