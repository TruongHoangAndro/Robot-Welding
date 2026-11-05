# WP-S S.5b — J2 Mode B screening (positioner angle table for the Engineer) Implementation Plan

**Goal:** Pivot evidence for the TST-002 meeting (Fri 2026-11-06), pulled forward from `PLAN.md` T1.4 by Truong on
2026-11-04 ("run Mode B with 2–3 Engineer-entered positioner angles"). Step 1 = **screening only**: per positioner
angle θ (30° grid inside the configured limits, 0 included), the S.5 per-point sweep on the whole closed J2 seam
(ψ × 8 IK branches, F4, F5 5 mm, F9) with the part and holders rotated, the s-intervals with ≥ 1 valid node, the
continuous DP windows (one IK branch, |Δq| ≤ 5°, wrist 3 ±180° within the window) and the failure classes / pairs.
A coverage map and the 2- / 3-angle sets that cover 100 % (with overlap at the joins) are **suggestions** for
Truong; the angles are Engineer-entered data (DES-001 Rev 0.6 draft §4.3 step 3, B-12). The final Mode B run with
the chosen angles is **not** part of this plan.

**Spec:** `PLAN.md` §3 WP-S, T1.4; DES-001 Rev 0.6 DRAFT §4.3 steps 1–5, §6.3 (Mode B), B-06, B-12; TST-002 Rev 0.3
DRAFT §5.1 F1–F11, §5.3 C2, C3, C8, §6; CP-53 (pitch rule); S.5 plan `2026-11-04-ws-s5-j2-sweep.md`; S.6 plan
`2026-11-04-ws-s6-j1-mode-c.md` (set_theta, B-03 margin).

**Decisions taken in (lead / Truong, 2026-11-04):**
- Positioner angles are Engineer data; this run only screens. Suggestions are labelled as such.
- CP-53 pitch rule, lead decision 2026-11-04: the |Δq| ≤ 5° pitch refinement is applied on **scene-valid nodes only**
  (F4 ∧ F5 ∧ F9 = `NodeScore.valid`), as a parameter (`pitch_rule`: `kinematic` = S.5, `scene_valid` = this run),
  tested and recorded. S.5 records stay as they are.
- Clearance links TST-002 F5 set (head_body, head_mount, conduit; m6 open). H-BOX head. B-03 margin 3 mm Assumption.
- Positioner limits unknown (OI-27): the sim cell config `awm_description/config/positioner_limits.yaml` (read through
  `awm_bringup.cell_config`) is used and recorded as an Assumption. Limits are never changed.

## Global constraints

Agent `planner`, worktree `.worktrees/planner`, branch `ws-s4c-checks`, ROS_DOMAIN_ID 43, build / test only with this
worktree's `scripts/dx`. Write scope: `src/awm_path/`, this plan, `docs/generated/spike/s5b_*`. `awm_path.core` pure,
mm / deg, conversions only through `awm_core.units`. No part data in source (seam id, θ step, overlap, ψ cases in the
data file `docs/generated/spike/s5b_mode_b_input.yaml`). Frozen definitions not tuned. Records carry commit + dirty,
input hashes, scene hash, backends, θ list and limits, ψ case, head model, clearance links, pitch rule, params.

## Task 1 — pitch rule on scene-valid nodes (pure, CP-53)

**Files:** `core/s5.py`, `core/scoring.py`, tests `test_core_s5.py`, `test_core_scoring.py`.

- [ ] `max_label_steps(layers, scores=None, closed=False)`: with `scores`, only candidates with a valid node score are
  paired; `closed` adds the step last → first.
- [ ] `refine_pitch(..., rule='kinematic' | 'scene_valid', closed=False)`: `scene_valid` expects `sample(n)` →
  `(layers, scores)`; closed seams use n points at pitch L / n (doubling keeps the earlier points). Result records the
  rule. Default behaviour unchanged (S.5 tests stay green).
- [ ] `score_candidates(..., contact=None)`: precomputed F9 results per point (F9 depends only on the seam point in the
  part frame, not on θ or ψ), length checked.
- [ ] Commit `awm_path: CP-53 pitch rule on scene-valid nodes (parameter), closed seams; F9 results reusable`.

## Task 2 — Mode B screening helpers (pure)

**Files:** `core/mode_b.py`, test `test_core_mode_b.py`.

- [ ] `screening_thetas(lower, upper, step)` (multiples of the step inside the limits, 0 included) and
  `theta_classes` (geometrically equal angles θ ≡ θ + 360°, representative in (−180°, 180°]).
- [ ] `circular_runs(flags)`; `window_lengths(layers, scores, params)` — per start layer, the layers one continuous
  DP path reaches on the closed seam (doubled layers, ≤ n; all F3 / F11 rules of `ladder.solve`), optional start
  stride (recorded); `maximal_windows`; `window_intervals` (s ranges, may pass the seam start).
- [ ] `bins_covered(intervals, length, bin_mm)`; `cover_sets(windows_by_theta, length, overlap_mm, k)` — one window per
  angle, joins overlap ≥ `overlap_mm`; ranked by the smallest join overlap (largest first), then θ; `best_cover` when
  none covers.
- [ ] Commit `awm_path: S.5b Mode B screening helpers (windows, coverage map, covering angle sets)`.

## Task 3 — runner `s5b_run` (ROS, mock hardware only) + data file

**Files:** `spike_s5b.py`, `setup.py` entry point, test `test_spike_s5b.py` (pure parts), data file
`docs/generated/spike/s5b_mode_b_input.yaml`.

- [ ] Reads the S.5 `spike_seam.json` (hash recorded), the data file, the positioner limits; checks θ ≡ θ + 360°
  equivalence by FK of `positioner_flange` and runs one sweep per θ class.
- [ ] Per θ and ψ case: CP-53 refinement (`scene_valid`), S.3 sweep, S.4 scores with cached F9, classes / pairs, valid
  intervals, windows; per-θ table; coverage map; covering sets (suggestions).
- [ ] Records `s5b_mode_b_screening.json` + `.md` in `docs/generated/spike/`; candidates CSV per θ in the job folder.
- [ ] Commit `awm_path: S.5b Mode B screening runner (sim_mock) + Engineer data file`.

## Task 4 — run, verify, record

- [ ] Clean tree, Ψ-FREE then Ψ-WIRE (Ψ-FREE only if > ~2 h).
- [ ] `scripts/dx "colcon build --symlink-install && colcon test --packages-select awm_path awm_description awm_bringup
  && colcon test-result --verbose"` (stale results removed first).
- [ ] Commit `docs: S.5b J2 Mode B screening records`. STOP and report (AGENT_TEAM.md §3).
