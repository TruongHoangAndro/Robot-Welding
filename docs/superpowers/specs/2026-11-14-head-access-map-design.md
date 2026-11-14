# Head access map (planner level 1) — design

Date: 2026-11-14 · Branch: `g2-1-head-access-map` · Status: **direction decided by Truong Hoang on 2026-11-14
("do level 1 first"); details below for approval** · Change proposal: CP-72 in
`docs/generated/DES-001_change_proposal.md`.

This document is written for a new coding session. Read it together with `CLAUDE.md` (rules, environment, `scripts/dx`)
and `docs/generated/doc_review_2026-11-14.md` (why this work exists).

## 1. Goal

The product goal (Truong, 2026-11-14): software that uses the robot, the welding source and the rotary positioner to
weld any part designed in 3D, **following the seam without collision, reliably**.

Level 1 answers one question per seam, before any robot kinematics is computed:

> At every point of the seam, in which orientations can the weld head sit on the seam without touching the part or
> the fixture that turns with it?

Output: a **head access map** per seam, and a verdict "accessible / not accessible, blocked by which object at which
arc length". It is the first of four planned levels (2 = robot + positioner states, 3 = path search and smoothing,
4 = independent certification of the final trajectory). Levels 2–4 are **not** part of this document.

## 2. Why (evidence)

Collision-on spike on joint J2 of the golden part 564082 (TST-002, the collision-avoidance feasibility spike; records
in `docs/generated/spike/s5_results.md`, `s5b_mode_b_screening.md`; box head model H-BOX, Ø 20 mm nozzle, work and
travel angle 0):

- robot-only: arc 1 covered 0 %, arc 2 38.3 %; 557 s for the seam;
- 12 positioner angle classes screened in about 77 min: **no** angle gives a valid state for s ≈ 2–40 mm
  (`PROGRESS.md` row "WP-S S.5 … S.7": 43 % of the seam "blocked at every θ by head ↔ part / rotating holders");
- dominant contact pairs: `part<->nozzle_tip`, `part<->nozzle_body`, `part2_1<->head_body`, `jig_1<->head_body`.

The part and its holders turn with the positioner. The pose of the head **relative to the part** at a seam point is
fixed by the seam frame and the head orientation only; it does not depend on the positioner angle θ, on the IK branch
or on the robot. The present sweep (ψ × 8 IK branches × θ, one `move_group` service call per state) therefore repeats
the same head ↔ part check thousands of times and cannot see that the cause lies outside the robot.

## 3. Definitions

Frames and conventions are those of DES-001 Rev 0.5 §5.3 and CP-58: seam frame `F_i` at seam point i, given in
`positioner_flange` at θ = 0 by `seam_axis.json` (X = travel direction, Z = out of the material, beam = −Z).

**Head orientation variables** (all in degrees):

| Symbol | Name | Meaning | Rotation |
|---|---|---|---|
| α | work angle | tilt of the beam across the joint | about seam X |
| β | travel angle | tilt of the beam along the travel direction (push / drag) | about seam Y |
| ψ | tool yaw | rotation of the head about the beam axis | about tool Z |

**TCP pose in `positioner_flange`** (extends the formula of `awm_path.core.sync`, which is the case α = β = 0):

```
flange_T_tcp(i, α, β, ψ) = F_i · Rx(α) · Ry(β) · Rx(180°) · Rz(ψ)
flange_T_tool0           = flange_T_tcp · (tool0_T_tcp)⁻¹
```

`tool0_T_tcp` and the head geometry come from the head description in use (`awm_path.core.head`, head.yaml), never
from constants. The contact point (TCP) stays on the seam for every (α, β, ψ); `focus_offset_mm` = 0 in this slice.
The implementer must add one test that pins this composition order (same style as the existing `fk_tcp` order test).

**Head set** = the head links of the description: `nozzle_tip`, `nozzle_body`, `head_body`, `head_mount`, conduit.
**Rotating set** = part (collision mesh with per-triangle `face_id`) + every scene object whose parent frame is
`positioner_flange` (role `rotary_fixture`, CP-60 / CP-55) from D19 `placement.json` v2 and the cell scene manifest.

**A head pose (i, α, β, ψ) is accessible** when all three hold:

1. no collision between any head-set body and any rotating-set object (TST-002 Rev 0.3 draft F4);
2. clearance ≥ `weld_clearance_mm` (5 mm, DES-001 §5.4) between `head_body`, `head_mount`, conduit and every
   rotating-set object, **measured on unpadded geometry** (F5; the nozzle is excluded, it ends at the contact point);
3. the nozzle contact rule F9 as built (`awm_path.core.contact.contact_check`, CP-56 rule (b) `rising`): contact
   point on the seam faces, no other face rising towards the nozzle within the stand-off.

Not checked at level 1 (level 2): the arm, the static cell, self-collision head ↔ arm, joint limits, reach.

## 4. Why the decomposition is exact

Checks 1–3 involve only bodies rigidly attached to `tool0` and bodies rigidly attached to `positioner_flange`.
Their relative transform is `flange_T_tool0(i, α, β, ψ)`. A rotation of the positioner by θ moves both sets together.
So an inaccessible pose at level 1 is inaccessible for **every** θ and every IK branch: level 1 can prove
"this seam cannot be welded with this head and this fixture", which no amount of robot search can change.

Two reductions follow and must be used:

- the nozzle (`nozzle_tip`, `nozzle_body`) is a body of revolution about the beam axis, so its collision and the F9
  check depend only on (i, α, β). A wire guide that turns with ψ would break this (OI-15); the head record must say
  whether the nozzle is axisymmetric, and the reduction is refused when it is not;
- the ψ sweep of `head_body`, `head_mount` and conduit runs only for the (i, α, β) that passed the nozzle checks.

Limit of the statement: the conduit is modelled as a rigid capsule fixed to the head (Assumption, as today).

## 5. Inputs

| Input | File | Notes |
|---|---|---|
| Seam points and frames in the flange frame | job `seam_axis.json` (schema `v2/seam_axis`) | derived by M2 from D04; header hashes checked |
| Seam faces n1 / n2 | job `seams.json` (D04 v2) | for F9 |
| Part collision mesh + `face_id`, job scene objects | job `placement.json` (D19 v2) + meshes | `awm_path.core.scene_files.load_job_scene` |
| Cell objects on the flange | `cell/model/scene_manifest.json` | objects with parent frame `positioner_flange` only |
| Head geometry, `tool0_T_tcp`, nozzle | head description (`awm_path.core.head`, head.yaml, head meshes) | H-CAD is the target; H-BOX for the regression of §9 |
| Parameters | D16 (cell config) — for this slice a parameter record passed to the CLI and written into the output | see §7 |

Every input is read through the existing loaders with their hash and schema checks. A missing or stale input refuses
the run (no fallback). Design rule 2 holds: no seam id, part name or pose in source.

## 6. Algorithm

Per seam (or per arc of a seam, CP-53):

1. **Sample.** Seam points at pitch `access_pitch_mm` (default 1.0) taken from `seam_axis.json` (sub-sample; never
   re-fit the curve). First and last point always included.
2. **Nozzle pass.** For every (i, α, β) on the tilt grid: nozzle collision (check 1 for `nozzle_tip`, `nozzle_body`)
   and F9 (check 3). Result: `nozzle_ok[i, α, β]` and the blocking object / `face_id` when false.
3. **Body pass.** For every (i, α, β) with `nozzle_ok` and every ψ on the ψ grid: checks 1 and 2 for `head_body`,
   `head_mount`, conduit. Result: `access[i, α, β, ψ]`, and for a rejected pose the first blocking pair.
4. **Clearance value.** For accessible poses, the minimum distance head set ↔ rotating set (mm), at least on the
   nominal tilt (α = β = 0) and on request for all (cost / benefit to be measured; level 3 will use it as a cost).
5. **Corridor analysis.** Build a graph on accessible poses: (i, α, β, ψ) connects to (i+1, α', β', ψ') when
   |Δα|, |Δβ| ≤ one tilt step and |Δψ| ≤ `psi_link_deg` (default one ψ step; ψ is cyclic). Report:
   - coverage = share of points with at least one accessible pose, at nominal tilt and within each tilt band;
   - whether a connected corridor exists from the first to the last point (closed seam: back to the start);
   - the s ranges with no accessible pose, each with its blocking objects and part faces;
   - per point the smallest |tilt| that gives access.
   The corridor is a necessary condition for a weld path, not a path: level 3 chooses the path.

No partial acceptance: the map never marks a seam "accepted". DES-001 rule 4 is unchanged; the map is evidence and a
pruning input for the planner.

## 7. Parameters (Assumption defaults, recorded in every output)

| Parameter | Default | Source / status |
|---|---|---|
| `access_pitch_mm` | 1.0 | Assumption (TST-002 pitch 1–3 mm) |
| `psi_step_deg` | 5 | Assumption; ψ range 0–355, or the WPS ψ window when filler wire is used (E-05 open) |
| `work_angle_deg` grid | 0, ±5, ±10, ±15 | **analysis grid only**; the allowed band is a welding-engineer input (open, §11 Q1) |
| `travel_angle_deg` grid | 0, ±5, ±10, ±15 | same |
| `weld_clearance_mm` | 5 | DES-001 §5.4 (D16) |
| F9 parameters | as `ContactParams` today | CP-56 |

The report always shows the result for **nominal tilt only** first, then per band, so that a wider band is never
mistaken for an accepted process window.

## 8. Collision engine

Requirement, not a library choice: an **in-process** checker of one rigid body set against another at a given
relative transform, exact on the triangle meshes / convex hulls that the scene export already produces (FCL works
on meshes, never on B-Rep, DES-001 §5.4), returning collision yes / no, the contact pair, and a minimum distance on
request. It must not need a running `move_group`.

Proposed interface (pure Python, no rclpy, `awm_path/core/`):

```python
class RelPoseChecker(Protocol):
    name: str
    def check(self, tools_T: np.ndarray) -> list[RelCheck]:
        """tools_T: (N, 4, 4) flange_T_tool0 in mm. One RelCheck per pose: valid, pairs, min_distance_mm (optional)."""
```

Candidate implementation: FCL through a Python binding (`python-fcl`) or `coal` (hpp-fcl), installed in the venv of
`awm_dev` and recorded in `docker/Dockerfile` (CLAUDE.md environment rule). **First task of the plan: check what is
installable in `awm_dev` and measure it** (the container was stopped on 2026-11-14, so nothing was verified).
Clearance on unpadded geometry: either a distance query with the threshold, or convex bodies inflated by exactly the
clearance — whichever the cross-check of §9 A3 confirms.

Expected size (estimate, not measured): J2 at 1 mm pitch ≈ 84 points × 49 tilts = 4 116 nozzle checks, then at most
4 116 × 72 ψ ≈ 296 000 body poses. A signed-distance-field checker (numpy lookup of sphere sets) is the fallback if
the exact checker is too slow, and is expected to be needed at level 2; it is not designed here.

Units: mm and degrees in all data; metres only through `awm_core.units` where a library wants them (ADR-14).

## 9. Outputs and acceptance

**Outputs** (job folder, derived data like `seam_axis.json`, never edited by hand):

- `access_map.json` — schema `v2/access_map` (new): header (`schema_version`, `created_by`, `created_at`, `inputs`
  with SHA-256 of `seam_axis.json`, `seams.json`, `placement.json`, scene manifest, head record), parameters, checker
  name and version, per seam / arc: grids, the accessibility array (compact encoding), blocking pairs per rejected
  (i, α, β) and per rejected ψ run, clearance values, corridor verdict, s ranges without access. Stale when any input
  hash changes (same rule as `awm_core.staleness`).
- A Markdown report next to it (table per seam: coverage at nominal tilt and per band, blocked s ranges with objects
  and faces, minimum tilt map, run time).
- CLI: `python3 -m awm_path.cli access-map --job <dir> [--seams ...] [--out <dir>]`, exit codes as the `plan`
  command (0 done, 1 refused input, 2 seam not accessible).

**Acceptance tests** (TDD is required: all code is under `src/`):

| # | Test | Kind |
|---|---|---|
| A1 | Composition order of `flange_T_tcp(i, α, β, ψ)`; α = β = 0 equals the pose used by `awm_path.core.sync` | unit |
| A2 | θ invariance: the verdict of a head pose does not change when both sets are transformed by Rx(θ) for several θ | unit, synthetic |
| A3 | Cross-check against the existing `move_group` backend: ≥ 200 poses on the golden job, same validity and same head ↔ rotating-set pairs (differences listed and explained, not tuned away) | sim test |
| A4 | Synthetic scenes with a known answer: plate + wall (blocked ψ range computable by hand), tube in a fork (blocked tilt side), free tube (100 %) | unit |
| A5 | Nozzle reduction: results with and without the (i, α, β) reduction are identical for an axisymmetric nozzle; refused for a non-axisymmetric one | unit |
| A6 | Determinism: identical inputs give a byte-identical `access_map.json` apart from the header time | unit |
| A7 | Stale or missing input, schema violation → refused with the cause | unit |
| A8 | Regression against the spike: H-BOX, Ø 20, tilt 0 on J2 — the s ranges without access are compared with the S.5 / S.5b records (expected: about 3–39 mm blocked). A mismatch is a finding to explain | golden |
| A9 | Golden run with the real head CAD (H-CAD): J1 and J2 maps at nominal tilt and per band; run time recorded (target ≤ 60 s per seam, to be confirmed by measurement) | golden |

A9 is the result Truong needs: **is J2 accessible with the real head, and with how much tilt.**

## 10. Out of scope of this slice

Levels 2–4; any change to `awm_path.core.sync` (it will consume the map later to prune (ψ, α, β) per point); the
Web UI overlay (S4 colours by access) — CLI and report only; approach / retract; the static cell; automatic change
of seam start, split or fixture; any release rule.

## 11. Open questions (defaults used until answered)

| # | Question | Default in this slice | Who |
|---|---|---|---|
| Q1 | Allowed work / travel angle band for LightWELD laser welding (with and without filler wire) | none accepted: bands are reported as analysis only | Welding engineer via Truong |
| Q2 | Nozzle geometry of the real head: head.yaml `nozzle_len_mm` is 0 for H-CAD, so there is no nozzle body and F9 is refused today (no silent fallback) | the run refuses H-CAD until the Admin enters nozzle length / diameter in the head record; A9 waits for it | Truong / Khang Doan (OI-15) |
| Q3 | Collision library (python-fcl, coal, other) | decided by the first plan task from what installs in `awm_dev` and the A3 cross-check | Software |
| Q4 | Does a "not accessible" verdict block Plan in the UI, or only inform? | inform only; gate decided with level 3 | Truong |
| Q5 | Cross-check 4 of the job import is red for 564082 (OI-23, CP-62 override). Level 1 is a collision computation: may it run on a job whose check 4 is red? | yes — it checks only head ↔ rotating set, which check 4 does not concern; the override id is recorded in the output | Truong |

## 12. Architecture fit

- Module M4 (`awm_path`), functions A44 / A45 (reach and collision), later A17 (placement study) and the S4 screen.
- New derived data file `access_map.json` (no D-number yet; to be assigned with the consolidated baseline, CP-71).
- ADR-12 (ladder-graph planner) is kept; level 1 is a pre-computation in front of it. ADR-15 respected (no OCC).
- Safety: simulation and geometry only. No hardware, no controller, no laser event.

## 13. Notes for the coding session

- Branch `g2-1-head-access-map` was created from `cp64-jog-teach-home` (not from `GotoCode`), because the review edits
  of 2026-11-14 build on the CP-64 text in DES-001 Rev 0.5. Those edits (`PROGRESS.md`, change proposal, DES-001,
  `docs/generated/doc_review_2026-11-14.md`, this spec) are committed on this branch (2026-11-14, not pushed).
- `awm_dev` was not running on 2026-11-14: `docker start awm_dev` (never touch `ros2_jazzy_dev_env` or `supabase_*`).
- Golden data: `scripts/golden_rebuild.py` builds the golden job from the cell STEP (`AWM_CELL_STEP`); the golden
  tests read it through their environment variables (see `src/awm_path/test/test_golden_g13.py`).
- Next step by the workflow rules: write the implementation plan
  `docs/superpowers/plans/2026-11-14-g2-1-head-access-map.md` from this spec (writing-plans), then TDD.
- Stop and ask Truong when a task touches Q1, Q2, Q4 or anything outside §10.
