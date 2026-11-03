# Change proposal for APLSMN601-DES-001 Rev 0.4 → Rev 0.5

Status: proposal by the software agent, for review by Truong Hoang / Linh Tran. Date: 2026-10-27.
Base documents: DES-001 Rev 0.4 (software architecture), DES-003 Rev 0.4 (function register),
TST-002 Rev 0.2 (collision-avoidance feasibility spike). `docs/reference/` content is not edited.

The proposals CP-1…CP-8 of 2026-10-19 were written against Rev 0.2. Their status after Rev 0.4:

| Old CP | Subject | Status after Rev 0.4 |
|---|---|---|
| CP-1 | ROS 2 Jazzy in Docker | **Closed** — ADR-10 changed to Jazzy in Rev 0.3 |
| CP-2 | OCCT in a separate process (`awm_geometry`) | **Mostly adopted** (ADR-15). Open: pythonocc version → CP-12 |
| CP-3 | Job package cache files | **Partly adopted** (D19, D20 in §5.2). Open: `cache/model.brep`, `cache/collision.stl` names → CP-13 |
| CP-4 | Small corrections | Replaced by CP-14 |
| CP-5 | Partial seams (`range_m`) | **Adopted** as A22 "split into segments"; field must move to mm (`range_mm`) |
| CP-6 | D07 contains the checked joint path | **Adopted in DES-003** (D07 = "tool poses / joint path"); DES-001 §5.1 still says poses only → CP-15 |
| CP-7 | KDL + pick_ik groups | **Withdrawn** — replaced by ADR-12 (analytic IK, ψ × 8 branches, ladder graph) |
| CP-8 | Dense collision check of the whole program | **Still proposed** → CP-16 |

## CP-9 Z-axis convention (decided by Truong, 2026-10-27)

**Decision:** Z is positive upwards.
- `world` and `base`: Z up (REP 103, UR convention).
- Seam frame (A41): X = tangent (travel direction), **Z = bisector of the face normals n1, n2, pointing out of the
  material (up, from the joint towards the head)**, Y = Z × X.
- Beam / tool axis: from the head into the joint = **−Z of the seam frame**. In `tcp_laser`, the tool Z axis is the beam
  direction (UR / ROS tool convention), so `tcp_laser.z = −seam.z` at the nominal angles.

**Conflict resolved:** TST-002 §4 D1 says "Z = −bisector"; DES-001 §4.3 and DES-003 A41 say "Z = bisector (out of the
joint)". Both are consistent once TST-002 is read as the beam axis.

**Text change:** DES-001 §4.3 step 1, DES-003 A41, TST-002 §4 D1: add "Z points out of the material (up); the beam
axis is −Z".

## CP-10 TCP definition and nozzle contact (OI-13, wiki L-001; left to the software team)

**Facts:** the LightWELD emits only when the nozzle / wire guide touches the grounded workpiece (part-contact circuit;
source: wiki L-001 quoting the LightWELD slides 6–7, not yet re-read here; DES-001 §6.2 lists a "workpiece-clamp
contact" interlock). TST-002 §3.1 places the TCP 10 mm beyond the
nozzle tip; TST-002 C2 asks for ≥ 5 mm nozzle clearance; DES-001 §5.4 lets only the TCP link touch the seam faces.
These cannot all hold: with the TCP 10 mm beyond the tip the nozzle never touches the part and the laser cannot fire.

**Proposal (Assumption until the IPG automation documentation, OI-2, confirms it):**

| Item | Proposal |
|---|---|
| TCP `tcp_laser` | On the beam axis **at the nozzle / wire tip**, i.e. the contact point on the seam |
| Focus offset | D06 field `focus_offset_mm` (default 0) moves the TCP along the beam axis; positive = away from the part |
| Collision links | Nozzle split into `nozzle_tip` (last 5 mm) and `nozzle_body` (rest) |
| ACM | `nozzle_tip` ↔ part: contact allowed (no padding). `nozzle_body`, head body, mount, conduit ↔ part / fixture / clamps: not allowed, 3 mm padding |
| Clearance targets | TST-002 C2 "≥ 5 mm" applies to head body, mount and conduit only; `nozzle_body` must stay free of contact with 3 mm padding |
| Contact force | Phase 1 plans the tip exactly on the surface (0 mm pre-load). Whether a compliant (spring) nozzle or force control is needed is a new open item (OI-22) |
| Approach / retract | Unchanged, 30–50 mm along the beam axis; the last approach millimetres and the first retract millimetres are the only motions with tip contact |

**Text change:** DES-001 §5.3 (`tcp_laser`), §5.4 (ACM row), OI-13 → closed by this decision; new OI-22; TST-002 §3.1
TCP row and C2.

## CP-11 Golden part file is a single fused solid

`docs/3D/564082_Solid.step` (AP214, mm) is **one solid**: a bent tube Ø 26.7 / Ø 21.0 mm, about 150 mm long
(straight – bend – straight – bend – straight), 12 faces, 30 edges. All edges between the straight and bent sections
are tangent-continuous (no groove, no fillet). TST-002 §2 describes "564082 Elbow assembly (3 parts)".

Consequences: A11 "split into solids" has nothing to split; the butt joints between the parts can still be picked as
the circular edges at the straight / bend transitions (each split into a 1/4 + 3/4 arc), but the two face normals are
equal there, so the seam frame Z is the surface normal. **Request:** the 3-body assembly STEP (wiki lists "564082 ×3"
in `StepFiles`) so that the real joints exist as separate bodies.

**Update 2026-10-27:** the 3-body assembly is inside the system STEP `docs/3D/APLSMN601_Automated Weldment System.step`
(component `564082-Assembly-3 parts`): `296783_PIPE NIPPLE` (4 287 mm³), `564082-Solid` (36 204 mm³) and
`564082-Elbow` (12 843 mm³). `564082_Solid.step` is `564082-Solid` + `564082-Elbow` fused (49 046 mm³ = sum). The
assembly has two closed butt joints, both Ø 26.67 mm outer (r 13.335): J1 nipple ↔ solid, J2 solid ↔ elbow. The
golden part for Sim-1 is this assembly, not the fused file (decided by Truong, 2026-10-27).

## CP-12 pythonocc-core version (wiki L-004)

ADR-15 and TST-002 E1 name pythonocc-core 7.9 / Python 3.12; the working environment is 8.0.1 / Python 3.10 (conda
`awm_occ`), tested on the generated parts and on 564082. Proposal: ADR-15 → "pythonocc-core ≥ 7.9 in its own conda
environment; tested with 8.0.1".

## CP-13 Job package cache names (§5.2)

Add `cache/model.brep` (D02 cache), `cache/collision.stl` (collision mesh, A13) next to `cache/view/` (D20).

## CP-14 Inconsistencies inside the Rev 0.4 document set

| # | Where | Issue | Proposal |
|---|---|---|---|
| 1 | DES-001 §4.3 para 2 | "reuses the MoveIt configuration and pick_ik setup of the Tac demo" — contradicts ADR-12 (analytic IK, 8 branches) | "reuses the MoveIt configuration; IK enumeration with analytic UR IK (ADR-12)" |
| 2 | DES-001 / DES-003 / TST-002 | Three clearance values: padding 3 mm (§5.4), head ≥ 5 mm (C2), air move ≥ 10 mm (C4) | Keep all three, each with its purpose, in D16 (`padding_mm`, `weld_clearance_mm`, `air_clearance_mm`) |
| 3 | DES-003 A48 vs TST-002 C3 | Wrist-3 ±270° soft limit vs ±180° of start (wiki L-007) | ±270° absolute soft limit (A48, D16) and ±180° travel within one seam (C3) — two different checks |
| 4 | DES-003 Data Items D09 / D10 | Format "URScript / ROS2 trajectory"; DES-001: JSON (`program.schema.json`) | "JSON (ROS2 joint-trajectory segments + events); URScript export = fallback" |
| 5 | DES-001 §4.2 `awm_bringup` vs CLAUDE.md | `sim` = URSim vs `sim` = mock hardware (wiki L-005) | Launch names `sim_mock` (Sim-1) and `sim_ursim` (Sim-2) |
| 6 | DES-001 §1.3 | Reference revisions outdated (wiki L-002) | Update in Rev 0.5 |
| 7 | DES-003 sheet "A0 Overview Phase 1" | Team pipeline A1–A4 with STL input and pose "X, Y, Z, Q1, Q2, Q3" (wiki L-012) | Rename P1–P4, input STEP, pose convention stated (mm + rotation vector or quaternion) |
| 8 | DES-003 A74 "Not in sim" | The chain releases with scope *simulation* (report only) for Sim-1 | Add "simulation release (not valid for production)" to A74 Sim note |
| 9 | Wiki L-003 | Says DES-003 Rev 0.3 history has "units = m"; the history says "units = mm". "units in m" appears only in DES-001 Appendix A (Tracker row) | Correct L-003 in the wiki |

## CP-15 D07 content (§5.1)

DES-001 §5.1 D07: "Weld path (part frame)" → "Tool poses (part frame, mm) **and the joint path chosen by A44**,
tied to the calibration hash". M6 reuses this joint path after an FK check instead of re-solving IK (IK enumeration
plus graph search must not be repeated with a possibly different result).

## CP-16 Verification resolution (former CP-8)

MoveIt validates trajectory waypoints only. M4 / M6 / A71 check every segment densely (≤ 0.02 rad joint step, plus
the 1–3 mm seam pitch) before release. §12 Correctness: "dense collision check of the complete program".

## CP-17 Sim-1 end point (decided by Truong, 2026-10-27)

Sim-1 ends at A71: program D09 + simulation report D15. No production release D10 is produced in Sim-1
(A72–A74 "Not in sim" stays).

**Sim-2 input (decided by Truong, 2026-10-27): simulation-scope release (CP-14 #8).** After A71 passes, M7 writes
D10 with `scope: simulation`. The runtime chain (M8 / M9) in simulation mode accepts only `scope: simulation`; with
real hardware it accepts only `scope: production`, which requires A72 / A73 records and Admin approval. Sim-2
therefore exercises the same release and hash checks as production. Text change: DES-003 A74 Sim stage "Not in sim"
→ "Sim-1: simulation-scope release only"; DES-001 §5.1 D10 adds the `scope` field.

## CP-18 Calibration before path planning (decided by Truong, 2026-10-27)

**Decision:** A4 plans with the active calibration D08. In the simulation stages D08 is the nominal calibration taken
from the cell CAD (CP-23); on hardware, A51 / A52 are done before A4.

**Consequence:** D07 records the D08 hash (CP-15). When the active D08 changes, D07 and everything after it are stale
and must be planned again. Proposal (not decided): allow a fast "re-anchor" that keeps the ψ / IK-branch sequence of
the old D07, re-solves IK with the new D08 and re-runs every check (reach, collision, Δq, wrist-3); any failure →
full re-plan.

**Text change:** DES-001 §4.3 (inputs of M4 include D08), §5.1 D07, §5.2 rule "any change … calibration".

## CP-19 Tack welds are made by hand (decided by Truong, 2026-10-27)

**Decision:** the operator tack-welds the parts by hand after they are clamped on the fixture. The robot does not
tack in Phase 1.

**Consequences:**
- DES-003 A22 / DES-001 §4.2 M2 "sequence incl. tacks" → "sequence" (no robot tacks in Phase 1).
- A81 operator sequence: load parts → clamp → tack by hand → Start request.
- Tack beads are not in the CAD. With nozzle contact (CP-10) the nozzle tip rides over them: add to OI-22
  (compliant nozzle / contact force). Tacking can move the part: input to A53 / A54.
- Where and with which tool tacks are made (inside the Class 4 enclosure?) is a process and safety question for
  REQ-002, not for the software.

## CP-20 Rotary positioner in Phase 1 (decided by Truong, 2026-10-27; closes OI-20)

**Decision:** the 1-axis rotary positioner of the mechanical design is part of Phase 1. This replaces Truong's earlier
conditional position of the same day. The approval of the Engineering Manager is to be recorded in DES-001 Rev 0.5.

**Hardware as modelled (system STEP, Fusion):** motor `60CM30X`, coupling `GSACL25-8-12`, `Rotary Shaft`, two
timing-belt stages (`HTPK20S5M150` pulleys, `Timing Belts`), holders `Part1` / `Part2` / `Part3` on the `Jig`.
Rotation axis horizontal (CAD X). On the golden part, J1 (nipple ↔ solid) is coaxial with the axis; J2 (solid ↔ elbow)
is 76.6 mm off the axis with its own axis perpendicular to it.

**Proposed Phase 1 motion modes (no synchronised 7-axis interpolation, §6.3):**

| Mode | Use | Motion during emission | Sequence |
|---|---|---|---|
| C — positioner weld | Seams coaxial with the axis (J1) | Robot at rest; positioner turns at ω = travel speed / seam radius, plus closing overlap | Positioner at rest → WeldStart → emission on (PLC) → M9 runs the positioner weld segment → positioner at rest → WeldEnd → emission off |
| B — rotate, stop, weld | Other seams (J2) | Positioner at rest at a planned angle; robot welds the arc segment | As today (§6.2) |
| A1 / A2 — continuous 7-axis | — | — | Phase 2 |

**Ownership (decided by Truong, 2026-10-27): ROS 2 controls the positioner motor.** This is the ADR-11 design moved
into Phase 1:
- `awm_positioner`: ros2_control hardware interface for the joint `positioner_joint`, loaded in the **same
  controller manager** as the UR hardware; Phase 1 uses its own `positioner_trajectory_controller`
  (joint_trajectory_controller). Sim-1 / Sim-2: `mock_components` for this joint (ADR-13).
- M9 executes positioner segments exactly like robot segments (it is motion, not a process event). M10 / PLC keep
  only the LightWELD.
- Recommended drive (OI-14): closed-loop drive with encoder on EtherCAT (CiA 402) through `ethercat_driver_ros2`, on
  its own IPC network port, PREEMPT_RT kernel (already planned, §8). It keeps the path open to continuous 7-axis
  motion (Option A1, Phase 2) without changing the software structure.
- Safety (hardware, independent of the IPC): drive STO wired into the safety chain (E-stop, door); drive ready / fault
  hard-wired to the PLC so the PLC switches emission off on a positioner fault; the PLC heartbeat watchdog (§6.1)
  also covers a ROS crash during a Mode C weld.
- Rule 6 holds: WeldStart / WeldEnd only while robot **and** positioner are at rest. Speed-scaling rule extended: a
  Mode C weld segment starts only at 100 % positioner speed (no override on weld segments), any deviation → WeldEnd +
  FAULT.

**Software impact:**
- D14: revolute joint `positioner_joint`, rotating links (shaft side, holders), frames `positioner_axis` /
  `positioner_flange`; fixture and part are children of `positioner_flange`. Sim-1: mock joint. Sim-2: URSim has no
  positioner → `mock_components` joint in the same controller manager as `ur_robot_driver`.
- M4: A49 moves to Phase 1. Mode C: one robot pose per seam, collision / clearance of head vs. the rotating part and
  holders checked over the full angle range. Mode B: positioner angle per segment from a discrete set, searched
  together with ψ × IK branches. A47: Mode C speed = WPS travel speed / seam radius.
- D07 / D09: points and segments carry the positioner angle (deg); D09 gets positioner segments and the Mode C weld
  event.
- M5: A55 (positioner axis calibration) moves to Phase 1; in simulation nominal from CAD.
- TST-002 C8: report coverage robot-only and with the positioner.
- Open (OI-14, narrowed): motor driver and its fieldbus (EtherCAT proposed), encoder or open loop, belt backlash. Accuracy check:
  angular error δ gives r·δ at the seam; at r = 13.335 mm, 0.1 mm ⇔ 0.43°.

**Text change:** DES-001 §1.2, §4.2 (`awm_positioner` Phase 1), §6.2 (Mode C sequence), §6.3, ADR-11, OI-14, OI-20
(closed); DES-003 A49, A55 → Phase 1 with Sim stage "Sim-1 MoveIt mock".

## CP-21 Seam picking by Engineer only (decided by Truong, 2026-10-27)

**Decision:** only the Engineer role picks and edits seams. **Text change:** DES-001 §4.5 S2 "Roles" → "Engineer";
§7.3 Operator row: delete "seam picking (S2) only with the 'Seam setup' permission"; §7.4 `/jobs/{id}/seams` PUT →
Engineer.

## CP-22 Part placement flow (A15) (decided by Truong, 2026-10-27)

**Problem:** A15 sits in A1, but a reach check needs the seams (A2) and the planner (A4); and the software placement
must match the physical fixture, so it cannot be moved freely once the fixture exists. Seams (D04) and process data
(D06) are in the `part` frame, so a placement change makes only D19 → D07 → D09 stale, never the seam picking.

**Decision:** combined flow for a new part number:

1. A1 import STEP (no placement needed for seam picking).
2. A2 the Engineer picks seams.
3. **Placement study**: candidate placements are evaluated with the planner. Sim-1: candidates entered by hand by the
   Engineer, full A4 run per candidate (PA1). Later (R3 Smart Path): automatic search over orientations with a coarse
   planner setting and a ranked reach table (PA2).
4. Mechanical design builds the fixture for the chosen placement (Fusion).
5. **Import placement from the fixture assembly** (CP-23): D19 is fixed by the fixture CAD (PA3). A placement change
   after this step requires a new fixture revision.
6. A3, then A4 full planning, A6, A7.

**Data:** D19 gets `source` = `study` | `fixture_cad`, plus fixture assembly file name, revision and SHA-256 when
`fixture_cad`. A simulation-scope release (CP-17) accepts both; a production release requires `fixture_cad`.

**Text change:** DES-003 A15 split into "Placement study" (after A2) and "Import placement from fixture CAD" (new
function ID to be assigned in DES-003 Rev 0.5); DES-001 §4.2 M1 / M4, §4.3, §5.1 D19.

## CP-23 Cell and fixture geometry from Fusion STEP assemblies (decided by Truong, 2026-10-27)

**Decision:** nominal cell geometry comes from Fusion (the CAD tool of the mechanical team, confirmed 2026-10-27), exported as STEP assemblies and read by `awm_geometry`
(pythonocc XCAF: component tree, names, placements). No CAD → URDF plug-in.

| Assembly | Content | Produces |
|---|---|---|
| Cell assembly (once per cell revision) | Pismo frame, table, robot mounting position, enclosure parts within reach | D14 static links + collision meshes, `world_T_base` |
| Fixture assembly (one per part number / fixture revision) | Fixture, clamps, the job part | D19 placement + collision scene |
| Head assembly | LightWELD head, mount, nozzle, conduit section (when IPG CAD arrives, OI-15) | Head collision links, nominal `tool0_T_tcp` |

**Rules:**
- The UR5e robot itself is **not** taken from CAD: kinematics, limits and collision meshes come from `ur_description`.
  The CAD gives only the base mounting pose.
- Export: STEP AP214 or AP242, assembly structure kept, units mm.
- Frames are given by **marker components**: a small part with a fixed name whose part origin is the frame
  (component placements survive STEP export; Fusion construction geometry and joint origins are not exported — to be verified).
  Names: `REF_WORLD`, `REF_UR_BASE` (aligned with the UR controller `base` frame, which is `base_link` rotated 180°
  about Z in `ur_description`), `REF_FIXTURE`, `REF_PART` (origin of the job STEP), `REF_TOOL0`, `REF_TCP`.
- Import checks: every required marker present exactly once; the part in the fixture assembly matches the job STEP
  (fingerprint / volume / bounding box); units mm; otherwise the import is refused with a clear message.
- The nominal calibration D08 of the simulation stages (CP-18) is computed from the markers (`base_T_fixture`,
  `tool0_T_tcp`). On hardware, A51 / A52 measurements replace it.

**Next step:** mechanical design exports one simple test assembly (table + fixture box + part + markers) to check
that marker placements and names survive the Fusion STEP export.

**Update 2026-10-27:** the first system STEP (`docs/3D/APLSMN601_Automated Weldment System.step`, 98 MB, AP214, mm)
was exported by Autodesk Translation Framework (Fusion). Read with pythonocc XCAF in
56 s: 269 component nodes, product names and placements kept (instance names get a `:n` suffix). Findings:
- **Y is up** in this file (casters at the lowest Y, robot above the frame). The import must apply a fixed
  CAD → `world` rotation (CAD Y → world Z); the CAD tool / export axis must be stated per file.
- No marker components yet. The UR base pose can be read from the UR CAD base body (`C-1000257`), but the UR CAD
  axis convention vs the controller `base` frame must be confirmed (marker `REF_UR_BASE` still needed).
- `ASSY, Welding tool` appears twice at the same pose (under `UR5E/UR7E` and at top level): duplicate geometry.
- The LightWELD head body is a surface model (212 shells, BRepCheck invalid): usable for collision only after
  tessellation / convex hulls, as planned in §5.4.
- The upper laser enclosure is not in the model (only the lower frame `2600001` and top plate).

**Text change:** DES-001 §5.3 (frames: source of nominal values), §5.4 (data sources table), §4.2 M1 / `awm_description`.

---

# Change proposal DES-001 Rev 0.5 → Rev 0.6 (architecture review, 2026-10-30)

Status: proposal by the software agent, for review by Truong Hoang / Linh Tran. Date: 2026-10-30.
Base: DES-001 Rev 0.5 (`docs/generated/release_rev0.5/`, working baseline, unchanged).
Draft: `docs/generated/drafts/APLSMN601-DES-001_Software_Architecture_Rev0.6_DRAFT.md`, built by
`docs/generated/drafts/make_des001_rev06.py` from the Rev 0.5 release (checked text replacements).
DES-003 is not changed in this step; its impact is listed per CP and goes into DES-003 Rev 0.6 after the decisions.

Review summary: the architecture principles are sound (offline / online split, PLC owns the laser, data-driven job
package with hashes, simulate first, single mm ↔ m adapter, release scopes). Four high risks: (1) the core planner
(ADR-12) is still unproven while other work builds on it; (2) absolute accuracy of offline programming from CAD is
not addressed; (3) the software view of safety is thin, mainly for Mode C (positioner turns during emission);
(4) scope versus schedule and the lack of an independent human review.

| CP | Subject | Kind | Needs decision | DES-001 Rev 0.6 | DES-003 impact |
|---|---|---|---|---|---|
| CP-24 | Accuracy chain: calibrated UR kinematics, part-frame correction hook | Design | Yes (Truong) | ADR-18, AD5, M5, 5.3, R14 | A51/A53/A54 text, D08 content |
| CP-25 | First error budget | Analysis | Values: welding engineer (OI-30) | 12.2 | A54 |
| CP-26 | Safety concept, software view | Design | PLr: safety engineer (OI-31) | 12.1, 1.4 | — |
| CP-27 | Watchdog also stops the positioner; Mode C burn-through check in PLC | Design | Yes (PLC / safety) | 6.1 | A82, A84 |
| CP-28 | Planning as asynchronous, reproducible run | Design | Yes (Truong) | ADR-19, 4.3, D07 | D07 content |
| CP-29 | Schema migration, atomic writes, job lock, release manifest | Design | Yes (Truong) | ADR-20, 5.2, D10 | D10 content |
| CP-30 | `awm_api` split into gateway + ROS bridge | Design | Yes (Truong) | ADR-21, 4.2 | — |
| CP-31 | Test strategy, observability, measurable quality targets | Design | Review | 12, 12.3, 12.4, D09, D12 | D09, D12 content |
| CP-32 | Web API rules: `/api/v1`, OpenAPI contract, error model, `202` + run id | Design | Review | 7.4 | — |
| CP-33 | Phase 1 positioner interface sized for Modes B / C only | Design | Yes (Truong) | ADR-22 | — |
| CP-34 | Sim-1 thin slice | Scope | Yes (Truong / PM) | Principle 9, OI-35 | "Sim stage" column |
| CP-35 | Human review gate for safety-relevant code | Governance | Yes (Engineering Manager) | Principle 10, ADR-23, OI-32 | — |

## CP-24 Accuracy chain (ADR-18)

**Finding:** AD5 asks for ±0.1–0.2 mm. The UR5e repeatability is ±0.03 mm (Fact, data sheet), but offline
programming from CAD depends on **absolute** accuracy. With the nominal `ur_description` kinematics this error is not
known and is expected to be larger than the band (Assumption, to be measured). Rev 0.5 does not mention the
robot-specific calibrated kinematics that UR stores in each controller. Part tolerance and tack distortion (CP-19) add
error, while seam tracking (A94) is Phase 2 and touch sensing is R4.

**Proposal:** on hardware, extract the calibrated kinematics with `ur_calibration` and use them in D14 / D08 (hash in
D08); simulation stays `NOMINAL`. `fixture_T_part` = nominal · correction; the correction comes from A53 (identity for
the fixed jig, measured for touch-off or a sensor) without changes in M4 / M6. Whether touch-off moves from R4 to R2 is
decided by the error budget (CP-25).

## CP-25 Error budget (12.2)

**Proposal:** a table of sources E1…E9 with Fact / Assumption / [TBD] marks and a decision rule, reviewed at every
calibration (A54). Only E1 is a Fact today. Values to be filled by OI-30 together with OI-16 (allowed seam deviation).

## CP-26 Safety concept, software view (12.1)

**Finding:** REQ-002 is still planned; DES-001 has no allocation of safety functions. Mode C lets the positioner turn
during emission under ROS 2 control.
**Proposal:** state that the IPC software is not a safety-related part; allocate SF1…SF6 to hardware / PLC / UR safety
configuration; PLr from REQ-002 (OI-31).

## CP-27 Watchdog and Mode C burn-through check (6.1)

**Proposal:** on IPC heartbeat loss the PLC switches emission off **and** removes the positioner drive enable. During a
Mode C weld segment the PLC switches emission off when the drive does not report motion in time, and raises a fault
when the positioner moves outside a weld or index segment.

## CP-28 Planning runs (ADR-19)

**Finding:** planning is long, started from a Web UI by several users, on one `move_group`; Rev 0.5 does not define
queueing, cancellation, job locking or reproducibility.
**Proposal:** `run_id`, one queue per `move_group`, job lock while a run is active, cancel, recorded seed, software
version and D16 / scene hashes in D07. Same inputs → same D07 (regression test).

## CP-29 Data integrity (ADR-20, 5.2)

**Proposal:** `schema_version` = `MAJOR.MINOR` with migration scripts in `awm_core` for MAJOR changes; atomic writes
(temp file + rename) and one writer per job; `release.json` holds a manifest with the SHA-256 of every released file,
checked by the runtime; archive copy on the shared drive. Git-backed job repository kept as a Phase 2 option.

## CP-30 `awm_api` split (ADR-21)

**Proposal:** FastAPI gateway process (HTTP / WebSocket, roles) and a separate rclpy bridge node; the bridge is the only
ROS client of the UI. Avoids mixing the asyncio loop and rclpy executors in one process.

## CP-31 Test strategy, observability, quality targets (12, 12.3, 12.4)

**Proposal:** test levels unit → contract → planner regression (golden part, reproducibility) → `sim_mock` integration
→ URSim → HIL with the PLC → FAT; negative test for every rejection rule. Structured logs with `job_id` / `run_id` /
`seam_id`, `/diagnostics`, software version stamps in D07, D09, D12, D18, daily backup with a restore test. New quality
rows: reproducibility, performance (≤ 30 s per seam, ≤ 5 min per job — Assumption), integrity, observability.

## CP-32 Web API rules (7.4)

**Proposal:** `/api/v1` prefix; generated OpenAPI is the contract and is reviewed like `awm_interfaces`; one error model
(RFC 9457 problem details); long operations return `202` + run id; new endpoints `/jobs/{id}/runs` (GET / DELETE).

## CP-33 Phase 1 positioner interface (ADR-22)

**Finding:** ADR-11 picks EtherCAT + PREEMPT_RT so that Option A1 (continuous 7-axis motion) stays open for Phase 2.
Phase 1 Modes B / C need no tight robot ↔ positioner synchronisation. OI-14 (drive, fieldbus) is still open.
**Proposal:** keep ROS 2 control (`awm_positioner`, ros2_control) but choose the Phase 1 drive interface on Phase 1
needs; the drive may run its own position / velocity profile. EtherCAT + A1 remains the Phase 2 target. This is a
detail of ADR-11, not a reversal; needs Truong's decision.

## CP-34 Sim-1 thin slice (principle 9)

**Finding:** about 5 weeks to the Phase 1 target, 13 modules, 10 screens, positioner, cell CAD import, release flow.
**Proposal:** Sim-1 = golden part 564082, J1 Mode C, J2 robot-only, screens S2 + S4 with a minimal job list; S3,
S5–S10 and automatic A18 after Sim-1 unless needed. Align PM-004 (OI-35).

## CP-35 Human review gate (ADR-23)

**Finding:** all functions are owned by the software agent (OI-28) and "Reviewed by" is [TBD]. The cell has a Class 4
laser and moving axes.
**Proposal:** CODEOWNERS and a protected branch; a named engineer approves every merge in M8, M9, M10,
`awm_positioner`, `awm_interfaces`, the PLC program and D16 limits (OI-32). Function ownership (OI-28) is unchanged.

## Document corrections in Rev 0.6

- Header: "Approved by" marked pending; status = draft.
- ADR-12: past decision date (29 Oct) replaced by OI-29 (new TST-002 decision date).
- New OI-29…OI-35 (TST-002 date, error budget values, REQ-002 PLr, reviewers, launch-name alignment `sim` vs
  `sim_mock`, code migration Rev 0.2 → Rev 0.5/0.6, Sim-1 scope).
- Not done in this step (proposed for later): move decision log and status text out of DES-001 into this change
  proposal, so DES-001 describes only the architecture; redraw of the figures (OI-17); release names R2 / R4 (charter CR).

# Architecture Review Board 2026-10-30 (AI reviewers) -> DES-001 Rev 0.6

Status: board consensus applied to the Rev 0.6 draft by the software agent (technical editor), 2026-11-02. **The
board members were AI reviewer agents (roles, not people), run by the software agent. Their result is input for the
human review by Truong Hoang and Linh Tran; nothing here is decided until they accept it.** Record of the board:
`docs/generated/arch_review_board_2026-10-30.md`.

**Method.** Three AI reviewer roles (CTO / Chief Software Architect; VP Quality, Functional Safety & Manufacturing
Engineering; Program / Delivery Director) reviewed the Rev 0.6 draft and the code independently (round 1, 41
proposals), cross-examined each other (round 2, 15 new proposals, concessions and conflicts), and a chair merged the
result into 25 decisions (B-01…B-25), a CP review (CP-24…CP-35) and 8 escalations (E-01…E-08). The editor applied the
decisions to `make_des001_rev06.py`; the draft is rebuilt from the Rev 0.5 release. Board verdict: **approve with
conditions, 5 / 10** (all three members 5 / 10 after round 2). New CP numbers start at CP-36.

## Board decisions

| B-id | Title | Decision | Priority | CP |
|---|---|---|---|---|
| B-01 | TST-002 decision on Fri 2026-11-06 with frozen inputs | accept_modified | P0 | CP-36 (new) |
| B-02 | Joint-aware seam frames for welds between separate solids | accept_modified | P0 | CP-37 (new) |
| B-03 | Planner state space and acceptance metrics (unwrapped joints, labels, pitch rule, unpadded clearance, singularity metrics, tie-break, displacement-bound step) | accept_modified | P0 | CP-38 (new) |
| B-04 | Nozzle contact checked explicitly; no ACM exemption | accept_modified | P0 | CP-39 (new) |
| B-05 | ψ window per WPS (filler wire, tip type); spike reports both cases | accept_modified | P0 | CP-40 (new) |
| B-06 | M4 core behind KinematicsBackend / SceneBackend; independent M7 re-check; runtime dependency rule | accept_modified | P0 | CP-41 (new) |
| B-07 | One owner per scene object (D14 / D19), shared scene manifest, default-deny CAD mapping | accept_modified | P0 | CP-42 (new) |
| B-08 | Migration order; T0.1 limited to kept code; D16 and D08 split | accept_modified | P0 | CP-43 (new) |
| B-09 | Immutable per-revision release bundle, one active pointer per scope | accept_modified | P0 | CP-29 (modified) |
| B-10 | One planning-context record, one compare function, policy table | accept | P0 | CP-44 (new) |
| B-11 | CLI subprocess job runner; no gateway / bridge split; achievable reproducibility | accept_modified | P0 | CP-28 (modified) |
| B-12 | Ordered Sim-1 slice: J1 Mode C first, J2 robot-only "100 % or rejected" | accept_modified | P0 | CP-34 (modified) |
| B-13 | Staged Web UI; v2 data contracts frozen 2026-11-04 | accept_modified | P0 | CP-32 (modified) |
| B-14 | Delivery stages Sim-1…HW-2; one DES-001 revision per gate; disposition table | accept_modified | P0 | CP-45 (new) |
| B-15 | D09 is the exact commanded trajectory; M9 sends it verbatim | accept | P1 | CP-46 (new) |
| B-16 | Release scope `trial`; production as promotion of the same bundle | escalate (E-02) | P1 | CP-47 (new, Proposed) |
| B-17 | Typed `awm_interfaces` v1; emission tied to axis motion; M8 heartbeat; no automatic retry; EmissionLost | accept_modified | P1 | CP-27 (modified) |
| B-18 | Named humans for PLC program, DES-002, safety, hardware execution; weld-event table; ADR-23 dates | escalate (E-03) | P1 | CP-35 (modified, Proposed) |
| B-19 | Minimal Sim-2 runtime: Python M9, M8-minimal, table-driven M10 sim, mixed-mode refusal, fault injection | accept_modified | P1 | CP-48 (new) |
| B-20 | ADR-22 conformance wording; Mode C ramps planned | accept_modified | P1 | CP-33 (modified) |
| B-21 | Accuracy chain: budget by direction and mode; IK refinement at HW-1; no runtime per-part correction | accept_modified | P1 | CP-24 (modified) |
| B-22 | Dated dependency register; customer demonstration agreement | accept | P1 | CP-49 (new) |
| B-23 | Company repository and CI runner; hardened IPC image before HW-1; network security | accept_modified | P1 | CP-50 (new) |
| B-24 | M11 / M12 deferred; Phase 1 weld log = M8 JSON lines | accept_modified | P2 | CP-31 (modified) |
| B-25 | Hardware-stage gate items: calibration change, TCP verification, UR safety configuration, installation block, seam re-match | accept_modified | P2 | CP-51 (new) |

All 25 decisions are written into the Rev 0.6 draft. B-16 and B-18 are escalations; their text is in the draft only
as **Proposed** amendments pending OI-38 / OI-39 — the decided ADR-17 and OI-28 stay in force until then.

## Outcome for CP-24…CP-35

| CP | Outcome | Draft change |
|---|---|---|
| CP-24 | Modified (B-21) | ADR-18 rewritten: calibrated kinematics mandatory on hardware, IK through KinematicsBackend with refinement before HW-1, checksum in the context record, no runtime per-part correction (OI-43); 5.3 paragraphs; M5 row |
| CP-25 | Modified (B-21) | 12.2 split into Robot weld / Mode C tables with Lateral / Along-beam / Allocation; E10–E13; E2 = Fact (bound); values due at the Sim-2 exit (OI-30) |
| CP-26 | Modified (B-18) | 12.1: sentence "SF3–SF5 are process interlocks…"; SF4 / SF5 / SF6 rows updated; full split at HW-1 |
| CP-27 | Modified (B-17) | 6.1 paragraph "Emission follows the welding axis"; 6.2 step 3 without retry; 7.1 weld_event typed; 7.2 new hard-wired rows |
| CP-28 | Modified (B-11) | ADR-19 = CLI subprocess runner, one slot, `409`, SIGTERM cancel; 4.3 "Planning runs"; reproducibility row |
| CP-29 | Modified (B-09) | ADR-20 = frozen per-revision bundle with active pointers; 5.2 rules and tree; D10 storage |
| CP-30 | **Withdrawn** | ADR-21 row kept (ADR-22 and ADR-23 are referenced) and marked **Rejected for Phase 1**; UI backend row = one FastAPI process |
| CP-31 | Modified (B-19, B-24, B-11) | 12.3 staged with fault-injection and HIL conformance rows; 12.4 staged; D15 validity statement; image digest |
| CP-32 | Modified (B-13) | 7.4 Stage column, runs endpoint, no jog, `production` refused until HW-2 gate code |
| CP-33 | Modified (B-20) | ADR-22 conformance wording; OI-14 purchase requirement; decide by 2026-11-20 |
| CP-34 | Modified (B-12, B-13) | Principle 9 ordered slice; 4.3 step 3 Mode B with entered angles; D19 `verified`; Section 11 A17 / A18 / A49 |
| CP-35 | Modified (B-18) | Principle 10 and ADR-23: effective 2026-11-23, wider scope, Sim-1 sign-off, PLC program needs a human author |

## Escalations (not decided — open items in the draft)

| E-id | OI | Question | Options | Board recommendation | Decider |
|---|---|---|---|---|---|
| E-01 | OI-37 | TST-002 decision on 2026-11-06 with frozen inputs, missing inputs = slip ≤ 3 days, not Pivot? | A yes; B decide with whatever exists, missing = Pivot; C no fixed date | A | Linh Tran, with Truong Hoang |
| E-02 | OI-38 | Amend ADR-17 with a `trial` scope; production = promotion of the same bundle? | A amend now; B decide at HW-1 gate; C keep, run A72 / A73 outside M8 / M9 | A (B fallback); C not acceptable | Truong Hoang |
| E-03 | OI-39 | Amend OI-28: named human PLC & safety engineer, commissioning engineer, LSO? | A amend (names by 2026-11-20, LSO before HW-2); B PLC program only; C keep | A | Linh Tran, with Truong Hoang |
| E-04 | OI-40 | Phase 1 deliverable on 2026-12-04? | A Sim-2 system, HW stages in PM-004; B move Phase 1 end after HW-2; C compress HW into December | A, with the customer agreement (OI-5) | Tac Huynh and Linh Tran |
| E-05 | OI-41 | Filler wire on J1 / J2, and from which side? | A autogenous, free ψ; B wire leading, ±15°; C unknown: report both, decide on wire case | Answer by 2026-11-04, else C | Truong Hoang (welding engineer, Khang Doan) |
| E-06 | OI-42 | Rebuild IPC / CI image from public `ros:jazzy` before HW-1? | A yes, in parallel, HW-1 entry; B demo image on the IPC; C switch now | A | Truong Hoang |
| E-07 | OI-43 | Runtime per-part IK re-solve as exception to design rule 1? | A no exception; B bounded exception; C offline re-plan per part | Decide at HW-1 gate; default A | Truong Hoang |
| E-08 | OI-44 | Demo part 564082 or 152056; part-detection sensor in Phase 1? | A 564082 only; B add 152056; C sensor in Phase 1 (own CP) | A, plus B if STEP by 2026-11-13 | Tac Huynh |

OI numbering note: B-25 names the requalification item OI-36, so the escalations start at OI-37 (B-14's reference
to "OI-37" for the Phase 1 deliverable is written as OI-40 in the draft).

## DES-003 and PLAN.md impacts (not applied in this step)

DES-003 Rev 0.6 (after the human decisions):
- Descriptions: A11, A21, A24, A41 (joint-aware frames, B-02); A41, A43, A44, A45, A48, A49 (state space, pitch,
  clearance, singularity, Mode C tolerance, B-03); A45, A71 (contact check, B-04); A31, A33, A42, A44, A49 (ψ window,
  B-05); A47, A62, A71, A83 (D09 as commanded trajectory, B-15); A72, A73, A74, A81 (trial scope, if E-02 = A);
  A63 (no automatic retry; table-driven M10 sim), A82, A84 (B-17, B-18); A85, A87, A91, A92, A95 (JSON-lines log,
  record types, B-24); A15, A18 (D14 / D19 boundary, default-deny, B-07); A51–A55 targets "from the error budget",
  A53 typed LOCATE_PART (B-21).
- New function A56 "Verify TCP" (Phase 1, R2, HW-2; B-25).
- Data items D04, D05, D06, D07, D08, D09, D10, D12, D13, D14, D16, D18, D19 descriptions.
- "Sim stage" column: A63 → "Sim-1 markers; Sim-2 M10 sim"; A81, A82, A84 → "Sim-2 (minimal)"; A17, A18, A49 per
  B-12; legend gains HW-1 / HW-2; align with the Stage column of DES-001 Section 11.
- Performers of A51–A55, A72, A73, A81–A84: named commissioning engineer (if E-03 = A).

PLAN.md (rebase on DES-001 Rev 0.5 / 0.6):
- §1: TST-002 row as the Sim-0 exit (2026-11-06); contract freeze 2026-11-04; HW-1 / HW-2 rows (dates TBD).
- §2: keep / replace column per package (closes OI-34).
- T0.1: scope = `awm_core` + kept code; schemas v2, context record, bundle layout, D16 / D08 split; regenerate test
  jobs. T0.2: scripted A18 with default-deny. T0.3: D14 / D19 boundary, "contact-model test" instead of ACM test.
  T0.5: `awm_interfaces` v1 typed. T0.6: URSim + `ur_robot_driver` moved into WP0; CI runner.
- WP1: spike = M4 core behind backends (2–5 Nov); pitch rule; pin `ur-analytic-ik`. T1.5 before T1.6; T1.6 split into
  entered angles (conditional) and automatic (R3).
- WP2: T2.1 joint-aware topology and scene manifest; T2.4 uses the CAD placement (`verified: false`); T2.6 writes the
  bundle layout.
- WP3: S1-min / S2 / S4-lite, from 2 Nov on the v2 example files; CLI job runner; units module and FK test.
- WP4: Python M9, M8-minimal, table-driven M10 sim, mixed-mode refusal, JSON-lines log, fault-injection script
  without retry; runbooks and support-bundle CLI.
- New §6 "Risks and triggers"; PROGRESS.md weekly dependency check.
- TST-002 Rev 0.3 (new revision copied into `docs/reference/` only from the Drive master): frozen inputs §2 / §4 / §5,
  entry checks, two ψ cases, two head models, outcome table §6, records per result.

# Changes found during the spike (from 2026-11-02)

## CP-52 Joint detection between solids without a general fuse (modifies B-02 / CP-37)

**Status:** accepted by Truong Hoang on 2026-11-02 (implementation evidence: WP-S S.1, TST-002 Rev 0.3 G1 PASS).
**Finding:** B-02 asked M1 to build a joint-aware weldment shape by a general fuse of the part solids (fuzzy value
0.05 mm), with every face keeping its source-solid id. S.1 shows that the fuse is not needed: the joint edges are found
directly between the separate solids (an edge of solid A coinciding with an edge of solid B within 0.05 mm), and at
each seam point n1 / n2 are the normals of the exterior faces of the two solids, i.e. the face next to the edge that
does not touch the other solid (`awm_geometry.joints`, solids read with `awm_geometry.assembly`).
**Result:** on the golden part 564082 (3 solids, J1 and J2) the seam-frame Z is radial within 0.000° at 12 points per
joint (per-solid rule of Rev 0.2: 45°). No boolean operation, no fuzzy-value tuning, every normal keeps its source
solid by construction.
**Proposal:** DES-001 Rev 0.6 M1 row: "joint detection between solids (general fuse; each face keeps its source-solid
id, B-02)" → "joint detection between solids (coincident edges within 0.05 mm, n1 / n2 from the exterior faces of
each solid; no general fuse; B-02 as modified by CP-52)". 4.3 step 1 is unchanged. PLAN.md S.1 text and the board
record line "M1 adds a joint-aware fused weldment shape" follow. The 0.05 mm coincidence tolerance is an
**Assumption** (tolerance of the Fusion export), to be revisited if a customer STEP has gaps or overlaps at a joint;
the tube-rule fallback (n1 = n2 = radial) stays documented in TST-002 Rev 0.3 §3.2.
**DES-003 impact:** A11 / A21 / A41 descriptions use "joint detection" instead of "fused weldment shape".

## CP-53 Closed seams welded robot-only are split into arcs; pitch refined to the joint step (TST-002 F1, C3, C8)

**Status:** accepted by Truong Hoang on 2026-11-03 (J2 = 2 arcs; the number of arcs is entered by the Engineer in the
UI, not chosen by software — Truong, 2026-11-03); pitch rule decided by the lead within F1 (≤ ceiling).
**Finding (WP-S S.3):** a closed 360° seam welded by the robot alone makes the tool orientation turn once about the
seam axis. That loop is not contractible in SO(3) (generator of π1(SO(3)) = Z2), while the forward-kinematics image of
a joint path that returns to the same unwrapped state is contractible. Hence **at fixed ψ** some joint must make a net
360° turn (topological fact); F11 (wrist 3 ±180° within one seam, fiber / conduit) and F3 (closure on the same
first-layer state) reject every such path. With ψ allowed to vary, a net ±360° ψ unwinding would make the loop
contractible; with the 10° ψ grid (step > Δq_max 5°) this did not happen on the synthetic r 13.3 mm placement
(all 36 ψ, 288 points: no path, open or closed) — an **empirical** result for that placement, not a proof (review of
S.3). Ψ-WIRE (±15°) cannot unwind 360°. Measured: ψ 90°, 288 points, the DP
stops at layer 145 on wrist-3 travel; without F11 it closes only on q6 + 360°. Second finding: at the F1 pitch
(1.16 mm, r 13.3 mm) joint steps reach 13°; at 0.29 mm they are ≤ 3.3°.
**Decision:** (1) The Engineer decides whether a seam is split: seam attribute `arcs` (integer ≥ 1, default 1) and
`arc_overlap_mm` in D04, entered in the seam editor S2 (A22 split). The software never splits a seam by itself; it plans
every arc (equal lengths from the seam start), accepts the seam only when every arc is 100 %, and otherwise rejects the
failing arc(s) with their failing points so the Engineer can change the number of arcs, the start point or the
direction. Emission stops between arcs (WeldEnd / WeldStart at rest, DES-001 §6.2). For the spike, J2 is entered with
2 arcs (each ≤ 180° + overlap); C2 / C3 apply per arc; C8 "2 segments" gates J2. A closed seam with `arcs` = 1 welded
robot-only is rejected (reason recorded: net 360° joint turn needed, F3 / F11).
Mode C (J1) is unaffected: the robot is at rest. (2) F1 is a ceiling: the planner refines the pitch until consecutive
IK solutions of one label move ≤ Δq_max and records the pitch used.
**Open:** weld quality at the arc restart / overlap (welding engineer; OI-11 / OI-16); whether a 360° wrist-3 turn is
acceptable for the fiber and wire conduit (Khang Doan / Nghia Nguyen) — if yes, a later CP can relax F11.
**F6 wording (S.3 review):** the shoulder-singularity metric is the distance |u| of the wrist centre from the plane of
the joint-1 and joint-2 axes (≥ 50 mm, Assumption); the literal "distance from the base Z axis" is always ≥ d4 =
133.3 mm on the UR5e and could never trigger. TST-002 Rev 0.3 F6 row updated.
**DES-001 impact:** §4.3 step 4 / 5 note "closed seams welded robot-only are split into arcs (CP-53)"; A22 (split) and
A42 (overlap) descriptions; TST-002 Rev 0.3 F1, C3, C8 rows updated in the draft.

