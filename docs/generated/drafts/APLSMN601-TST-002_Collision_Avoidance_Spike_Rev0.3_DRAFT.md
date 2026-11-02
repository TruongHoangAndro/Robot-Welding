# APLSMN601-TST-002 — Collision Avoidance Feasibility Spike

| Item | Value |
|---|---|
| Project | APLSMN601 Automated Welding Machine |
| Document | APLSMN601-TST-002 Collision Avoidance Feasibility Spike — scope, frozen inputs, pass/fail criteria and results |
| Rev | 0.3 (draft) — inputs frozen before the run; results section filled after the run |
| Date | 2026-11-02 |
| Prepared by | Truong Hoang (draft by the software agent) |
| Decision | Fri 2026-11-06, chaired by Linh Tran (Engineering Manager) — proposed, DES-001 OI-29 / escalation E-01 |
| Related | APLSMN601-DES-001 Rev 0.6 DRAFT (ADR-12, §4.3, §5.4, D16; board decisions B-01…B-06), DES-003 Rev 0.5 (A41, A44, A45, A49), Architecture Review Board record of 2026-10-30 (`docs/generated/arch_review_board_2026-10-30.md`, AI reviewer agents) |

## Revision history
| Rev | Date | Description | By |
|---|---|---|---|
| 0.1 | 2026-10-21 | First draft (MIG torch) | Truong Hoang |
| 0.2 | 2026-10-22 | Tool changed to IPG LightWELD 1000 head; simulation only; C8 added; data in mm | Truong Hoang |
| 0.3 (draft) | 2026-11-02 | Spike not run in the 23–29 Oct window; new window 2–5 Nov, decision 6 Nov. Inputs frozen before the run (board B-01…B-05): joint-aware seam frames for the 3-solid golden part with an entry gate; pitch rule; seam frame Z = bisector (DES-001 Rev 0.5 convention; D1 "Z = −bisector" read as the beam axis); state space with unwrapped joints and configuration labels; clearance on unpadded geometry; singularity metrics; deterministic tie-break; Mode C coaxiality tolerance; nozzle contact checked explicitly (no ACM exemption); two ψ cases and two head models; gate scope = J2 robot-only; outcome "not decided"; records per result | Truong Hoang (draft by the software agent) |

## 1. Purpose
Prove, within a fixed timebox and in simulation only, that the planned collision-avoidance method (DES-001 ADR-12)
works on the golden part before the full implementation is committed:
- full collision scene (part mesh + fixture + positioner holders + LightWELD head, mount and conduit) in MoveIt2, and
- search over the redundant rotation about the beam axis (ψ) and the 8 UR5e IK branches.

Second purpose: report how much of a closed 360° seam the UR5e covers alone (C8), as input to the Mode B decision
for J2 (DES-001 OI-27: robot-only first, Mode B only if robot-only < 100 %).

Output: a Go / Pivot / No-Go / not-decided result with evidence (Section 6).

The spike code is written as the M4 core library (DES-001 B-06): pure Python, mm, behind `KinematicsBackend` and
`SceneBackend`. It is not throw-away code.

## 2. Scope
**In scope**
- Part: `564082-Assembly-3 parts` from the cell CAD `docs/3D/APLSMN601_Automated Weldment System.step` — three
  separate solids (pipe nipple, solid, elbow). Not `docs/3D/564082_Solid.step` (older fused copy).
- Seams: **J2** (solid ↔ elbow, closed butt joint, Ø 26.67 mm, 76.6 mm off the positioner axis) robot-only = the
  gating seam. **J1** (nipple ↔ solid, coaxial with the positioner axis) in Mode C = reported, does not gate.
- Simulation only: MoveIt2 with mock hardware in the `awm_dev` container (ROS 2 Jazzy).
- Scripted seam input (edge ids + n1 / n2 from OCCT). No Web UI.
- Sweep: ψ in 10° steps (5° inside narrow windows) × 8 IK branches, for the two ψ cases of §3.3.
- Two head models (§3.1): CAD hulls from the cell CAD and the conservative box.
- All data in mm and degrees (ADR-14); conversion to m / rad only through `awm_core.units`.

**Out of scope**
- Production UI, final schemas (only the v2 drafts of 2026-11-04), wobble, PLC / LightWELD handshake.
- Work / travel angle sweep (only in the Pivot path, Section 7).
- Mode B search (entered angles only, after the decision; DES-001 B-12).
- URSim and real robot motion.
- C6 (placement robustness) may follow after the decision; it does not block it.

## 3. Inputs and equipment
| # | Input | Source | Status (2026-11-02) |
|---|---|---|---|
| I1 | Golden part, 3 solids | Cell CAD (Fusion STEP, Y up) | Available |
| I2 | LightWELD head + mount geometry | Cell CAD (head = surface model → convex hulls); IPG CAD still to request (OI-15) | Available as hulls; nozzle tip shape open |
| I3 | Fixture, holders, positioner rotating set | Cell CAD | Available; which holders rotate = OI-27, due 2026-11-06 — until then all holders on `positioner_flange` (conservative) |
| I4 | Part pose in the cell, robot base pose | Cell CAD component placements (no `REF_*` markers yet, OI-23) | **Assumption**, recorded per result |
| I5 | Filler wire on J1 / J2, wire side | Welding engineer via Truong (escalation E-05) | Open, due 2026-11-04 — both ψ cases are run anyway |
| E1 | `awm_dev` (Jazzy, MoveIt2), pythonocc-core 8.0.1 in conda `awm_occ`, `ur-analytic-ik` (version pinned in `docker/Dockerfile`) | — | pythonocc available; ur-analytic-ik to install (T0.6) |

### 3.1 Head models (both are run)
| Model | Content | Status |
|---|---|---|
| H-CAD | Convex hulls of the head and mount from the cell CAD; conduit capsule Ø 30 × 300 mm at the head rear | Head shape: Fact (CAD); conduit: Assumption |
| H-BOX | Box 194 × 35 × 381 mm (data-sheet envelope) + mount box + conduit capsule | Conservative envelope (Fact: data sheet dimensions) |
| Nozzle | Cylinder Ø 20 × 40 mm at the beam exit, split into `nozzle_tip` (last 5 mm) and `nozzle_body` | **Assumption** until IPG data (I2) |
| TCP | On the beam axis at the nozzle tip = contact point on the seam (DES-001 §5.3, decision 2026-10-27; replaces "10 mm beyond the tip" of Rev 0.2); `focus_offset_mm` = 0 | Assumption until IPG data (OI-2) |

A seam that passes with H-BOX very likely passes with the real head. A result that differs between H-CAD and H-BOX is
reported with the blocking link.

### 3.2 Seam frames for welds between separate solids (entry gate)
- M1 builds a joint-aware weldment shape by general fuse of the part solids (fuzzy value 0.05 mm, **Assumption**).
  Every face keeps its source-solid id. n1 / n2 are taken from the exterior faces of the two solids at the joint.
- Fallback if the fuse is not ready on 2026-11-03: tube rule for coaxial tube butt joints, n1 = n2 = radial, marked
  **Assumption** in the results.
- Frame: X = tangent (travel), **Z = bisector of n1 / n2, out of the material** (for a tube butt joint = the radial
  surface normal), Y = Z × X. The beam axis is −Z. (Rev 0.2 D1 "Z = −bisector" = the beam axis.)
- **Entry gate G1:** on the real 3-solid assembly, the seam-frame Z of J1 and J2 is radial within 0.5° at every
  point. If G1 fails, the sweep does not start.

### 3.3 Tool-yaw (ψ) cases
| Case | ψ range | Use |
|---|---|---|
| Ψ-FREE | 0–350°, 10° steps | Autogenous weld, round / cone tip |
| Ψ-WIRE | ±15° about the wire-leading direction (**Assumption**), 5° steps | Filler wire fed from one side |

Both cases come from the same run. The decision record states which case it covers (I5). With filler wire, a wire
guide collision link turning with ψ is added once its geometry is known (OI-15).

## 4. Procedure (2–5 Nov 2026)
| Day | Step | Output |
|---|---|---|
| Mon 2 Nov | Joint-aware shape or tube rule (§3.2); seam extraction J1 / J2; discretisation by the pitch rule (§5.1) | `spike_seam.json` (mm), frame plots; **G1** result |
| Mon 2 – Tue 3 Nov | Scene: part mesh (chord ≤ 0.1 mm near the seams, `face_id` per triangle), holders, fixture, H-CAD and H-BOX; ACM without any head ↔ part entry (§5.2) | Scene files; scene hash |
| Tue 3 Nov | C1 scene sanity incl. contact-zone poses | Test log; **G2** result |
| Tue 3 – Wed 4 Nov | Candidate sweep per point: ψ (both cases) × 8 branches → IK, validity, unpadded clearance, singularity metrics, contact check | `candidates.csv` |
| Wed 4 Nov | Ladder-graph DP on J2 robot-only; J1 Mode C check over 360° + overlap | `spike_path.json` (mm / deg), C8 coverage |
| Thu 5 Nov | Approach / retract along the beam axis (40 mm), OMPL air move to home; timing; Section 11 filled | Results table, decision proposal |
| Fri 6 Nov | Decision meeting | Decision record |

The software agent runs the spike; Truong Hoang runs the checks.

## 5. Frozen definitions and pass / fail criteria

### 5.1 Definitions (frozen 2026-11-02; values in D16 `cell.yaml` where marked)
| ID | Definition | Value |
|---|---|---|
| F1 | Pitch | Δs ≤ min(3 mm, Δq_max / seam-frame rotation rate); ≈ 1.16 mm at r = 13.3 mm |
| F2 | Graph node | (point, ψ, configuration label from the joint values, **unwrapped** joint vector); 2π offsets enumerated at the first layer within the joint limits |
| F3 | Edge | same configuration label and max \|Δq\| ≤ 5° (Δq_max) on unwrapped values; closed seams: the overlap reconnects to the first-layer state |
| F4 | Validity | Collision check with 3 mm padding on every link except `nozzle_tip` |
| F5 | Clearance (C2, C4) | Distance on **unpadded** geometry, by distance query or by a separate validity check with head / mount / conduit inflated by exactly the clearance. Method recorded |
| F6 | Singularity (D16) | wrist \|sin q5\| ≥ sin 8.6°; elbow \|sin q3\| ≥ sin 5° (Assumption); shoulder: wrist centre ≥ 50 mm from the base Z axis (Assumption). Per-point values in `candidates.csv` |
| F7 | Tie-break | lowest cost, then lowest ψ index, then configuration-label order |
| F8 | Mode C | seam axis within 0.05 mm / 0.1° of the positioner axis (D16, Assumption); otherwise Mode B / robot-only |
| F9 | Contact | No ACM entry between any head link and the part. Nozzle collision geometry stops s_c = 0.3 mm (Assumption) short of the contact point. Per weld state: contact point(s) on the seam faces (faces of n1 / n2) within tolerance; no other face within s_c |
| F10 | Dense check step | No robot or head point moves more than `max_check_displacement_mm` < 2 × (guaranteed clearance − margin) between checked states; positioner Δθ ≤ 2 · (c − margin) / r_max |
| F11 | Wrist 3 | ±270° absolute and ±180° within one seam, on unwrapped values |

### 5.2 Entry gates (a failure stops the sweep)
| ID | Gate | Pass |
|---|---|---|
| G1 | Seam frames J1 / J2 | Z radial within 0.5° at every point (§3.2) |
| G2 | C1 scene sanity | 10 / 10 colliding poses invalid; 10 / 10 free poses valid; contact-zone poses invalid: tip 0.5 mm into a non-seam face, rim into a corner wall, flat tip tilted 8° on a flat face |

### 5.3 Criteria
| ID | Criterion | Pass threshold | Gates the decision |
|---|---|---|---|
| C1 | Scene detects collisions correctly | G2 | Yes (entry) |
| C2 | Seam coverage | ≥ 1 valid candidate at **100 %** of the J2 points; head body / mount / conduit clearance ≥ 5 mm (F5); contact check F9 passed | Yes |
| C3 | Path continuity | One configuration label for the whole seam; max \|Δq\| ≤ 5° per step (F3); wrist 3 per F11 | Yes |
| C4 | Approach, retract, air move | Collision-free, clearance ≥ 10 mm unpadded to part / fixture / holders | Reported |
| C5 | Planning time | ≤ 30 s per seam on the dev PC, with the backend named | Yes (a fail → backend change, B-06, not Pivot) |
| C6 | Robustness to placement error | C2–C4 with the part shifted ±2 mm and ±1° per axis (6 runs) | After the decision |
| C8 | 360° coverage robot-only | % of J2 covered by one continuous path, and with 2 segments; per ψ case and head model | Reported (OI-27 input) |
| C9 | J1 Mode C | Head clearance ≥ 5 mm over 360° + overlap at one robot pose; contact check F9 | Reported |

## 6. Decision rules
| Result | Decision |
|---|---|
| G1 or G2 fails | **Stop and fix** — no result is valid until both pass |
| C1–C3 pass (C5 pass or a backend fix in sight) | **Go** — WP1 continues on 9 Nov |
| C3 or C5 fails only | **Go** — tune pitch / cost / backend; not a feasibility issue |
| C2 fails | **Pivot** — at most 3 working days of Section 7 steps 1–3 in MoveIt; J2 enters Sim-1 as "rejected with failing points"; Tesseract evaluated only after Sim-2 |
| Pivot steps fail and no option in Section 7 works | **No-Go** — Sim-1 = J1 Mode C only; Phase 1 scope escalated to Linh Tran / Tac Huynh |
| Inputs or tooling missing on Thu 5 Nov | **Not decided** — at most +3 working days; not a Pivot |
| C8 < 100 % with 2 segments | Mode B with entered angles for J2 (DES-001 B-12); no effect on Go / Pivot |

The decision record names the ψ case (Ψ-FREE / Ψ-WIRE) and the head model (H-CAD / H-BOX) that it covers.

## 7. Pivot path (if C2 fails)
1. Sweep work angle ±10° and travel angle ±10° (5° steps); re-run C2.
2. Log the blocking geometry (link, object, point). If only H-BOX blocks, decide on H-CAD and wait for the IPG CAD.
3. Evaluate in order: tilted head mount (15° / 30°), part orientation in the jig, holder relocation, robot base
   position, seam split, Mode B angles.
4. If nothing works: classify J2 as not reachable in Phase 1 and report to the Engineering Manager.

## 8. Roles (proposed)
| Role | Who |
|---|---|
| Spike execution (code, scene, sweep, DP) | Software agent (Claude Code) |
| Checks of the results, gate review | Truong Hoang |
| Filler wire / ψ window (I5) | Welding engineer, Khang Doan (via Truong) |
| Rotating holders, r_max (I3) | Nghia Nguyen (mechanical) |
| Decision | Linh Tran (chair) + Truong Hoang, 6 Nov |

## 9. Records
Code in Git (M4 core library). Per result, record: commit (and dirty flag), input hashes (part, cell, `cell.yaml`),
backend (`SceneBackend` implementation), ψ case, head model, clearance method (F5). Files: `spike_seam.json`,
`candidates.csv`, `spike_path.json`, gate and test logs, scene hash, screenshots. Store a copy in
`4 DRAWINGS AND DESIGN EVALUATION\4. SOFTWARE\5. Spike\` (proposed).

## 10. Risks and assumptions
| Type | Item |
|---|---|
| Assumption | Part and base poses from CAD component placements (no `REF_*` markers yet) |
| Assumption | Nozzle Ø 20 × 40 mm, s_c = 0.3 mm, TCP at the tip, ψ-wire window ±15° |
| Assumption | Elbow and shoulder singularity thresholds (F6), Mode C tolerance (F8), fuse fuzzy value (§3.2) |
| Risk | General fuse of the 3 solids fails or is slow → tube rule fallback (§3.2) |
| Risk | move_group service latency makes C5 fail → batched in-process backend (DES-001 B-06), not a Pivot |
| Risk | ur-analytic-ik install in the container delays day 1 → port of `ur_kin.cpp` |
| Risk | Fiber stiffness not modelled — wrist-3 limits and the conduit capsule used as proxy |

## 11. Results (filled after the run)
| ID | Result | Value | Pass / Fail | ψ case | Head model | Note |
|---|---|---|---|---|---|---|
| G1 | | | | — | — | |
| G2 / C1 | | | | — | both | |
| C2 | | | | | | |
| C3 | | | | | | |
| C4 | | | | | | |
| C5 | | | | | | backend: |
| C8 | | | | | | |
| C9 | | | | | | |

Decision: ______ Covers: ψ case ______ head model ______ Date: ______ Approved: ______
