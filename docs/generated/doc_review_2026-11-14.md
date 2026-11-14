# APLSMN601 — requirements, architecture and solution review, 2026-11-14

Requested by Truong Hoang. Three read-only reviewer agents (PM, QA, Dev) read the documents and the repository on
branch `cp64-jog-teach-home`; the lead agent checked their main claims and wrote this record. No code was changed and
no test was run for this review: every test count quoted by the reviewers comes from `PROGRESS.md`.

Abbreviations: R05 = DES-001 Rev 0.5 (`docs/generated/release_rev0.5/`, working baseline); R06 = DES-001 Rev 0.6 DRAFT
(`docs/generated/drafts/`, never issued); CPF = `docs/generated/DES-001_change_proposal.md`.

## 1. Output goal (restated by Truong, 2026-11-14)

> One software product that uses the robot, the welding source and the rotary positioner to weld **any part**
> (frames, pipe sections, …) that has been designed in 3D with SolidWorks or Fusion.

Reading of the goal, split into what the documents must answer:

| # | Element of the goal | Where it is covered today | Gap |
|---|---|---|---|
| G-1 | Any part, no code change per part | R05 AD2, design rule 2, job package §5.2 | No measurable statement: R05 §12 "≥ [TBD] jobs". Every result so far is on one part (564082, two Ø 26.67 mm butt joints) |
| G-2 | Part classes: frames, pipe sections | Seam schema has `butt`, `fillet`, `lap` | No fillet, lap, T or saddle (pipe-to-pipe) seam has been planned on a real part; no frame test part exists |
| G-3 | Size and mass of a part | — | No operating envelope anywhere: UR5e reach 850 mm and 5 kg payload, positioner payload and swing radius (OI-14 open), enclosure. "Any part" has no stated limit |
| G-4 | CAD from SolidWorks or Fusion | D01 = STEP AP203 / 214 / 242; ADR-16 Fusion assemblies | Only Fusion exports were read. No SolidWorks STEP was tested (assembly tree and names, up axis, multi-body weldment parts, gaps between bodies vs the joint-detection tolerance of CP-52) |
| G-5 | Robot + positioner weld a complete seam | CP-58 Goal 1, CP-59 `sync` | Done with collision off only. Collision-free motion (Goal 2) is not designed for a moving positioner |
| G-6 | Welding | Laser only in Phase 1 (LightWELD); MIG Phase 2 | WPS inputs missing (E-05). Feasible travel speed on the golden part is 2.39 mm/s (J2) and 6.96 mm/s (J1): heat-input feasibility not assessed by a welding engineer |
| G-7 | A fixture for every part | CP-60: fixture is a role in the job STEP | Fixture design per part is a mechanical task outside the software; placement study A17 deferred |
| G-8 | What "delivered" means in Phase 1 | PLAN.md: Sim-2 exit (E-04, not decided) | Not decided whether Phase 1 ends in simulation or on hardware |

## 2. Findings (consolidated)

Class: **DOC** = correction of an inconsistency or of an already decided item (no design change); **DEC** = needs a
decision by Truong (proposals CP-66…CP-71 in CPF); **TEST** = test or criterion to add to the plan.

### 2.1 Requirements and plan (PM)

| # | Sev. | Finding | Class |
|---|---|---|---|
| P1 | High | Sim-1 / Sim-2 exit criteria exist only in unapproved drafts (`PLAN.md` §1 "draft for review", R06 §2.3 "Proposed"); R05 has none; no SRS (REQ-001 "planned") | DEC (CP-68) |
| P2 | High | The Sim-1 exit in `PLAN.md` ("J1 Mode C; J2 robot-only") no longer matches CP-58 / CP-59 (`sync`, collision off first). Goal 1 outputs carry `collision_checked: false` and can never be released, but Sim-1 ends at a simulation release (CP-17) → Sim-1 cannot close before Goal 2 | DEC (CP-68) |
| P3 | High | Goal 2 (collision on) has no plan and no design; Sim-1 date is 2026-11-20. On 564082 cross-check 4 stays red until the cell CAD is fixed (OI-23, due only "before HW-1") and CP-62 forbids collision-on planning while it is red | DEC (CP-68, CP-70) |
| P4 | High | TST-002 decision never recorded: Rev 0.3 draft §11 empty; spike records show C2 FAIL (J2 0 % / 38.3 %), C5 557 s, C4 and C6 not run, all with the box head model. By TST-002 §6 this is Pivot. ADR-12 still "Proposed — decide after TST-002" | DEC (CP-69) |
| P5 | High | Sim-2 on 2026-12-04 is not credible: no `awm_cell`, `awm_executor`, `awm_plc_bridge`, `awm_positioner`, `awm_motion`; no URSim in `docker/`; `awm_interfaces` is still the MIG-typed v0; DES-002 weld-event table does not exist; no test for 7 joints under UR speed scaling | DEC (CP-68) |
| P6 | Medium | Escalations E-01, E-03…E-08 open past their dates (CPF "Escalations"); CP-57 and CP-65 proposed; CP-60 tolerances "open for approval" although implemented | DEC |
| P7 | Medium | `PLAN.md` (2026-11-02) and `PROGRESS.md` (2026-11-08) are behind: no Goal 1 / Goal 2, "WP0…WP4 not started", CP-63…65 missing; four branches unmerged (`g1-21`, `g1-23`, `g1-24`, `cp64`) | DOC after CP-68 |
| P8 | Medium | `CLAUDE.md` is stale in places: "Rev 0.4" in the header, table and package layout; "not approved yet" next to "accepted"; launch name `sim`; base branch `GotoCode` vs main `master`; Sim-0 "Git + CI" though no CI exists | DOC (Truong owns the file) |
| P9 | Low | `release_rev0.5/SHA256SUMS` no longer matches DES-001 (edited for CP-64 and for this review) | DOC |
| P10 | — | Risks not tracked: no risk register in the repo; single-source evidence (agent writes, tests and reviews); personal Git remote, no CI; ADR-23 human reviewers unnamed | DEC |

### 2.2 Verification (QA)

| # | Sev. | Finding | Class |
|---|---|---|---|
| Q1 | High | `TRACEABILITY.md`, `FINAL_REPORT.md`, `USER_GUIDE.md` date from 2026-10-19 (MIG chain, A23). `docs/generated/function_register_*.md` export Rev 0.4. No acceptance test found for A17, A25, A43 (C4 10 mm), A45 on the golden part, A71 report of the 7-axis chain | DOC + TEST |
| Q2 | High | All golden evidence is collision off; release tests that pass are the legacy v1 chain plus refusal tests. No golden test for collision-on planning or for a simulation release of a 7-axis program | TEST (after CP-68 / CP-70) |
| Q3 | High | Units rule (ADR-14): the scale-literal scan covers only the xacro files; legacy v1 code still converts ad hoc (`awm_path/planner.py:116`, `poses.py:108`, metre fields in `awm_params.yaml`). T0.1 source scan not done | TEST + DEC (remove or quarantine the v1 chain) |
| Q4 | High | LaserReady timeout: R05 §6.2 "one automatic retry, then FAULT"; `PLAN.md` T4.3 / T4.6 and R06 "no retry" (B-17) | DEC |
| Q5 | Medium | Contact rule written three ways: `CLAUDE.md` (TCP link may touch seam faces), R05 §5.4 (ACM for `nozzle_tip`), TST-002 Rev 0.3 F9 / CP-56 as built (no ACM, 0.3 mm stand-off, contact by `face_id`). The CAD head with nozzle length 0 has no contact link | DEC (confirm F9), then DOC |
| Q6 | Medium | Clearance 5 / 10 mm: R05 §5.4 does not say padded or unpadded geometry (R06: unpadded). Dense check: 0.02 rad (R05) vs displacement bound (R06 / F10); no test asserts the step | DOC + TEST |
| Q7 | Medium | TST-002 Rev 0.2 (in `docs/reference/`, "read first") says TCP 10 mm beyond the tip; R05 §5.3 says at the tip | DOC (Rev 0.3 from the Drive master) |
| Q8 | Medium | R05 §12 criteria not testable in simulation: accuracy ±0.1–0.2 mm, "FAT", "speed error < 5 %" without a reference | DEC (simulation-stage substitutes) |
| Q9 | Medium | Skips beyond the one allowed (`AWM_CELL_STEP not set`): `AWM_OCC_PYTHON`, `AWM_SPIKE_REPO`, `AWM_GOLDEN_REPO`, `importorskip` of rclpy / xacro / ur_analytic_ik, one xfail (check 4 red, by decision) | DOC (widen the rule) or TEST |
| Q10 | Medium | No table-driven role test per screen; no automated check of "no absolute paths", "no real IP", runtime rules (events at rest, 100 % scaling) | TEST |

### 2.3 Architecture and algorithms (Dev)

| # | Sev. | Finding | Class |
|---|---|---|---|
| D1 | High | No single current baseline: R05 holds CP-9…23 + CP-64; R06 holds the board decisions B-01…B-25 but none of CP-52…65; OI-29 / OI-30 mean different things in R05 and in R06 / CPF | DEC (CP-71) |
| D2 | High | CP-59 (`sync`) and CP-63 were decided but R05 still said "Phase 2" and "Engineer only" in about 20 places | DOC — **applied 2026-11-14** (§3) |
| D3 | High | Sync rules still to design: one speed-scaling value on 7 joints, PLC rule "emission follows the welding axis" with two moving axes, positioner-lag monitoring | DEC (new OI-31 in R05) |
| D4 | High | ψ is not a usable redundant DOF on a 10° grid with \|Δq\| ≤ 5° per step (`SyncParams`: `psi_step_deg` 10, `dq_max_deg` 5; CP-53 records that ψ never unwound). ψ is in effect constant per arc; the wire window (B-05) is not in `sync`; E-05 unanswered | DEC (CP-70) |
| D5 | High | Collision with a continuously moving θ has no design: state space θ lattice × ψ × 8 branches per point; the `move_group` backend measured 557 s per seam and 77 min for 12 θ classes; CP-57 (GPU pre-filter) undecided; batched backend not built; `point_status` has no collision / clearance / nozzle value; air moves are joint-interpolated and unchecked | DEC (CP-70) |
| D6 | Medium | Algorithm items that live only in code or in CPs: pitch refinement, cost weights (`w_up` 0.05), acceleration-aware maximum speed, weld ramp 5 mm/s² after WeldStart. Missing: closing overlap in `sync`, closure of a 360° seam, a check that the joint-space spline stays on the seam and holds the TCP speed between knots, deterministic air moves, a planning-time target | DOC + DEC |
| D7 | Medium | R05 §5.3 frame formula is wrong and stale: `… · tcp⁻¹` gives `base_T_tool0`, not `base_T_tcp`; `Rz(θ)` although the built positioner axis is +X; `flange_T_fixture · fixture_T_part` replaced by `positioner_flange_T_part` (CP-60). `seam_axis.json`, `roles.json`, `cad_check.json`, override record, `frames.yaml`, `head.yaml` have no D-number; §5.1 / §5.2 / ADR-16 still describe `REF_*`, `cad_map.yaml`, `fixture.step` | DOC after the CP-60 details are approved |
| D8 | Medium | D08 (calibration) is not in the v2 chain: the program generator refuses when an active D08 exists and uses nominal `frames.yaml`; no v2 schemas for D05, D06, D08, D15 | DEC (before Sim-2) |
| D9 | Medium | As built without a CP: `awm_program` / `awm_verify` / `awm_api` import `awm_path` internals (no `awm_motion`, B-06); `awm_verify/verify.py` is still the 6-joint Rev 0.2 chain, no frozen bundle (B-09); `awm_api` has no rclpy (matches B-11, contradicts R05 §4.2); S4 "Replay in RViz" sends an unreleased, collision-unchecked D09 to the mock controller without M8; controller rate 250 Hz vs 500 Hz in §6.3 | DEC / DOC |
| D10 | Medium | Board items never implemented: B-06, B-09, B-17, B-19, B-23; ADR-23 effective 2026-11-23 without named reviewers | DEC |

## 3. Corrections applied on 2026-11-14 (decided items only)

DES-001 Rev 0.5 (`docs/generated/release_rev0.5/`), 24 text replacements, no new design content:

- **CP-59** (`sync`, decided 2026-11-05): §1.2, §1.4 terms, §4.2 M4, §4.3 steps 3 and 5, §4.5 S4, §6.2 (events at rest
  of robot **and** positioner in every mode; speed-scaling paragraph), §6.3 title, Option A1 row, drive paragraph and a
  new "Decision 2026-11-05" paragraph with the three hardware gates, §7.1 (7-joint controller action), ADR-11, §11
  A49, Appendix A, new **OI-31** (`sync` on real hardware).
- **CP-63** (Admin ⊇ Engineer ⊇ Operator, decided 2026-11-09): §4.5 roles of S2–S4, §7.3 Engineer and Admin rows.

Not applied (waiting for a decision): everything marked DEC above; the CP-60 / CP-61 / CP-62 texts (CP-60 details still
"for approval"); the frame formula of §5.3; `CLAUDE.md`; `SHA256SUMS`.

## 4. Decisions asked from Truong

Proposals CP-66…CP-71 in CPF ("Review of 2026-11-14"):

1. **CP-66** Product goal statement and operating envelope of "any part".
2. **CP-67** Validation part set (pipe, frame, plate; one SolidWorks export) as the measure of "any part".
3. **CP-68** Sim-1 / Sim-2 exits redefined and re-dated; plan rebased on Goal 1 / Goal 2.
4. **CP-69** TST-002 outcome recorded (Pivot; `sync` + real head as the pivot path), ADR-12 status.
5. **CP-70** Goal 2 design questions: collision with a moving positioner, ψ resolution, time budget.
6. **CP-71** One baseline document: DES-001 Rev 0.6 = R06 + CP-52…CP-65, OI renumbered.

## 5. Not verified

No test, container or ROS command was run. The `.xlsx` function registers were read through their Markdown exports
only. Drive masters, PM-004, DES-002, DES-004 and decisions taken outside the repository are unknown to the reviewers.
The behaviour of `scaled_joint_trajectory_controller` with a 7th joint was not checked. The reviewers read the code at
docstring / grep level; "no test found" means not found by name.
